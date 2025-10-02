package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

// Road Runner core
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

// RR FTC utils
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/** MecanumDrive with Pinpoint localizer and Actions follower (center-origin friendly). */
@Config
public final class MecanumDrive {

    // ----- quick knobs -----
    public static double TICKS_PER_REV      = 537.7;
    public static double WHEEL_DIAMETER_IN  = 96.0 / 25.4;
    public static double TRACK_WIDTH_IN     = 14.0;
    public static double LATERAL_MULT       = 1.0; // set -1 if strafes reversed

    // follower params
    public static class Params {
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        public double inPerTick = 1;
        public double lateralInPerTick = inPerTick;
        public double trackWidthTicks = 0;

        public double kS = 0, kV = 0, kA = 0;

        public double maxWheelVel = 50;
        public double minProfileAccel = -30, maxProfileAccel = 50;

        public double maxAngVel = Math.PI, maxAngAccel = Math.PI;

        public double axialGain = 0.0, lateralGain = 0.0, headingGain = 0.0;
        public double axialVelGain = 0.0, lateralVelGain = 0.0, headingVelGain = 0.0;
    }
    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics;
    public final TurnConstraints defaultTurnConstraints;
    public final VelConstraint   defaultVelConstraint;
    public final AccelConstraint defaultAccelConstraint;

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public final VoltageSensor voltageSensor;
    public final LazyImu lazyImu;

    public final Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter     = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter   = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    // --------- optional drive-encoder localizer (not used with Pinpoint) ---------
    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;
        private Pose2d pose;

        public DriveLocalizer(Pose2d pose) {
            leftFront  = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBack   = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightBack  = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));
            imu = lazyImu.get();
            this.pose = pose;
        }

        @Override public void setPose(Pose2d pose) { this.pose = pose; }
        @Override public Pose2d getPose() { return pose; }

        @Override
        public PoseVelocity2d update() {
            PositionVelocityPair lf = leftFront.getPositionAndVelocity();
            PositionVelocityPair lb = leftBack .getPositionAndVelocity();
            PositionVelocityPair rb = rightBack.getPositionAndVelocity();
            PositionVelocityPair rf = rightFront.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS",
                    new MecanumLocalizerInputsMessage(lf, lb, rb, rf, angles));
            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;
                lastLeftFrontPos  = lf.position;
                lastLeftBackPos   = lb.position;
                lastRightBackPos  = rb.position;
                lastRightFrontPos = rf.position;
                lastHeading = heading;
                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{(lf.position - lastLeftFrontPos),  lf.velocity}).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{(lb.position - lastLeftBackPos),   lb.velocity}).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{(rb.position - lastRightBackPos),  rb.velocity}).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{(rf.position - lastRightFrontPos), rf.velocity}).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos  = lf.position;
            lastLeftBackPos   = lb.position;
            lastRightBackPos  = rb.position;
            lastRightFrontPos = rf.position;
            lastHeading = heading;

            pose = pose.plus(new Twist2d(twist.line.value(), headingDelta));
            return twist.velocity().value();
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack  = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack .setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack .setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu",
                new RevHubOrientationOnRobot(PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // compute params BEFORE building kinematics
        if (PARAMS.inPerTick == 1) {
            double wheelCircumferenceIn = Math.PI * WHEEL_DIAMETER_IN;
            PARAMS.inPerTick = wheelCircumferenceIn / TICKS_PER_REV;
            PARAMS.lateralInPerTick = PARAMS.inPerTick;
        }
        if (PARAMS.trackWidthTicks == 0) {
            PARAMS.trackWidthTicks = TRACK_WIDTH_IN / PARAMS.inPerTick;
        }

        // kinematics AFTER params (fixes yaw mixing)
        kinematics = new MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks,
                LATERAL_MULT * (PARAMS.inPerTick / PARAMS.lateralInPerTick));

        defaultTurnConstraints = new TurnConstraints(PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
        defaultVelConstraint   = new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                new AngularVelConstraint(PARAMS.maxAngVel)));
        defaultAccelConstraint = new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

        // Pinpoint localizer
        localizer = new PinpointLocalizer(hardwareMap, pose);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    /** Apply drive powers (-1..+1). */
    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels =
                kinematics.inverse(PoseVelocity2dDual.constant(powers, 1));

        double max = 1.0;
        for (DualNum<Time> w : wheelVels.all()) max = Math.max(max, Math.abs(w.get(0)));

        leftFront.setPower(wheelVels.leftFront.get(0)  / max);
        leftBack .setPower(wheelVels.leftBack .get(0)  / max);
        rightBack.setPower(wheelVels.rightBack.get(0)  / max);
        rightFront.setPower(wheelVels.rightFront.get(0) / max);
    }

    // ---------------- Actions follower wrappers ----------------
    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double now = seconds();
            double t = (beginTs < 0) ? 0 : now - beginTs;
            if (beginTs < 0) beginTs = now;

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack .setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            // power application (FF or fallback)
            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);

            double lf, lb, rb, rf;
            boolean noFF = (PARAMS.kS == 0 && PARAMS.kV == 0 && PARAMS.kA == 0);
            if (noFF) {
                double s = 1.0 / Math.max(1e-6, PARAMS.maxWheelVel);
                lf = clamp(wheelVels.leftFront.get(0)  * s, -1, 1);
                lb = clamp(wheelVels.leftBack .get(0)  * s, -1, 1);
                rb = clamp(wheelVels.rightBack.get(0)  * s, -1, 1);
                rf = clamp(wheelVels.rightFront.get(0) * s, -1, 1);
            } else {
                double v = voltageSensor.getVoltage();
                MotorFeedforward ff = new MotorFeedforward(
                        PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
                lf = ff.compute(wheelVels.leftFront)  / v;
                lb = ff.compute(wheelVels.leftBack )  / v;
                rb = ff.compute(wheelVels.rightBack)  / v;
                rf = ff.compute(wheelVels.rightFront) / v;
            }

            mecanumCommandWriter.write(new MecanumCommandMessage(voltageSensor.getVoltage(), lf, lb, rb, rf));

            leftFront.setPower(lf);
            leftBack .setPower(lb);
            rightBack.setPower(rb);
            rightFront.setPower(rf);

            // overlay
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);
            c.setStroke("#4CAF50");  // target
            drawRobot(c, txWorldTarget.value());
            c.setStroke("#3F51B5");  // current
            drawRobot(c, localizer.getPose());
            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;
        private double beginTs = -1;

        public TurnAction(TimeTurn turn) { this.turn = turn; }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double now = seconds();
            double t = (beginTs < 0) ? 0 : now - beginTs;
            if (beginTs < 0) beginTs = now;

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack .setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            // power application (FF or fallback)
            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);

            double lf, lb, rb, rf;
            boolean noFF = (PARAMS.kS == 0 && PARAMS.kV == 0 && PARAMS.kA == 0);
            if (noFF) {
                double s = 1.0 / Math.max(1e-6, PARAMS.maxWheelVel);
                lf = clamp(wheelVels.leftFront.get(0)  * s, -1, 1);
                lb = clamp(wheelVels.leftBack .get(0)  * s, -1, 1);
                rb = clamp(wheelVels.rightBack.get(0)  * s, -1, 1);
                rf = clamp(wheelVels.rightFront.get(0) * s, -1, 1);
            } else {
                double v = voltageSensor.getVoltage();
                MotorFeedforward ff = new MotorFeedforward(
                        PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
                lf = ff.compute(wheelVels.leftFront)  / v;
                lb = ff.compute(wheelVels.leftBack )  / v;
                rb = ff.compute(wheelVels.rightBack)  / v;
                rf = ff.compute(wheelVels.rightFront) / v;
            }

            mecanumCommandWriter.write(new MecanumCommandMessage(voltageSensor.getVoltage(), lf, lb, rb, rf));

            leftFront.setPower(lf);
            leftBack .setPower(lb);
            rightBack.setPower(rb);
            rightFront.setPower(rf);

            // overlay
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);
            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());
            c.setStroke("#3F51B5");
            drawRobot(c, localizer.getPose());
            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    // -------- helpers --------
    /** monotonic seconds without NanoClock dependency */
    private static double seconds() { return System.nanoTime() * 1e-9; }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());
        while (poseHistory.size() > 100) poseHistory.removeFirst();
        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));
        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;
            i++;
        }
        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    /** Minimal robot outline for Dashboard (≈9" square + heading tick). */
    private static void drawRobot(Canvas c, Pose2d p) {
        double halfLen = 9.0 / 2.0, halfWid = 9.0 / 2.0;
        double ch = Math.cos(p.heading.toDouble()), sh = Math.sin(p.heading.toDouble());

        double[][] pts = {
                {+halfLen, +halfWid},
                {+halfLen, -halfWid},
                {-halfLen, -halfWid},
                {-halfLen, +halfWid},
                {+halfLen, +halfWid}
        };
        double[] xs = new double[pts.length], ys = new double[pts.length];
        for (int i = 0; i < pts.length; i++) {
            double lx = pts[i][0], ly = pts[i][1];
            xs[i] = p.position.x + ch * lx - sh * ly;
            ys[i] = p.position.y + sh * lx + ch * ly;
        }
        c.setStrokeWidth(1);
        c.strokePolyline(xs, ys);

        double hx = p.position.x + ch * halfLen;
        double hy = p.position.y + sh * halfLen;
        c.strokeLine(p.position.x, p.position.y, hx, hy);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(1e-6, new ProfileParams(0.25, 0.1, 1e-2)),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
