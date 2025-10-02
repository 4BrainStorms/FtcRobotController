package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "AxisSanity (Pinpoint) v2", group = "debug")
public class AxisSanity extends LinearOpMode {

    // --- translation P (unchanged) ---
    private static final double K_POS = 0.045;
    private static final double LIN_CLAMP = 0.65;
    private static final double MIN_CMD = 0.07;

    // --- yaw P (stronger + min command) ---
    private static final double K_YAW = 0.08;                 // was 0.02
    private static final double YAW_CLAMP = 0.80;             // was 0.40
    private static final double YAW_MIN_CMD = 0.12;           // new: kick to beat stiction
    private static final double YAW_SIGN = 1.0;               // set to -1.0 if turn direction is reversed

    private static final double STOP_POS_IN = 1.0;
    private static final double STOP_YAW_RAD = Math.toRadians(2.0);
    private static final long   SEG_TIMEOUT_MS = 5000;

    private static final double STEP_IN = 10.0;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        telemetry.addLine("AxisSanity v2: FWD 10\", LEFT 10\", TURN +90°");
        while (opModeInInit()) {
            Pose2d pp = drive.localizer.getPose();
            telemetry.addData("Pinpoint", "x=%.2f y=%.2f θ=%.1f°",
                    pp.position.x, pp.position.y, Math.toDegrees(pp.heading.toDouble()));
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        if (isStopRequested()) return;

        Pose2d start = drive.localizer.getPose();

        // 1) Forward +10"
        Pose2d g1 = offsetByRobot(start, STEP_IN, 0.0, 0.0);
        goToPoseOpenLoop(drive, g1, SEG_TIMEOUT_MS);

        // 2) Left +10"
        Pose2d g2 = offsetByRobot(g1, 0.0, STEP_IN, 0.0);
        goToPoseOpenLoop(drive, g2, SEG_TIMEOUT_MS);

        // 3) Turn +90° at same spot (heading-only controller)
        double targetHeading = g2.heading.toDouble() + Math.toRadians(90);
        turnToHeadingOpenLoop(drive, targetHeading, SEG_TIMEOUT_MS);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
    }

    // --- move to global pose with translation P ---
    private void goToPoseOpenLoop(MecanumDrive drive, Pose2d goal, long timeoutMs) {
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < timeoutMs) {
            Pose2d pose = drive.localizer.getPose();

            double dx = goal.position.x - pose.position.x;
            double dy = goal.position.y - pose.position.y;

            double ch = Math.cos(pose.heading.toDouble());
            double sh = Math.sin(pose.heading.toDouble());
            double errFwd  =  ch*dx + sh*dy;
            double errLeft = -sh*dx + ch*dy;
            double errYaw  = wrap(goal.heading.toDouble() - pose.heading.toDouble());

            double fwd  = clampWithKick(K_POS * errFwd,  LIN_CLAMP, MIN_CMD);
            double left = clampWithKick(K_POS * errLeft, LIN_CLAMP, MIN_CMD);
            double yaw  = clampWithKick(YAW_SIGN * K_YAW * errYaw, YAW_CLAMP, YAW_MIN_CMD);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, left), yaw));
            drive.updatePoseEstimate();

            telemetry.addData("Pose", "x=%.1f y=%.1f θ=%.1f°",
                    pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Err", "f=%.2f l=%.2f yaw=%.1f°", errFwd, errLeft, Math.toDegrees(errYaw));
            telemetry.addData("Cmd", "f=%.2f l=%.2f w=%.2f", fwd, left, yaw);
            telemetry.update();

            if (Math.abs(errFwd) < STOP_POS_IN &&
                    Math.abs(errLeft) < STOP_POS_IN &&
                    Math.abs(errYaw) < STOP_YAW_RAD) break;

            sleep(10);
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        drive.updatePoseEstimate();
        sleep(80);
    }

    // --- heading-only turn (fwd/left = 0) ---
    private void turnToHeadingOpenLoop(MecanumDrive drive, double targetHeadingRad, long timeoutMs) {
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < timeoutMs) {
            double cur = drive.localizer.getPose().heading.toDouble();
            double errYaw = wrap(targetHeadingRad - cur);

            double yaw = clampWithKick(YAW_SIGN * K_YAW * errYaw, YAW_CLAMP, YAW_MIN_CMD);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), yaw));
            drive.updatePoseEstimate();

            telemetry.addData("Turn", "cur=%.1f° tgt=%.1f° err=%.1f°",
                    Math.toDegrees(cur), Math.toDegrees(targetHeadingRad), Math.toDegrees(errYaw));
            telemetry.addData("Cmd", "w=%.3f", yaw);
            telemetry.update();

            if (Math.abs(errYaw) < STOP_YAW_RAD) break;
            sleep(10);
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        drive.updatePoseEstimate();
        sleep(80);
    }

    // robot-frame offset → field-frame goal
    private static Pose2d offsetByRobot(Pose2d base, double fwd, double left, double dThetaRad) {
        double ch = Math.cos(base.heading.toDouble()), sh = Math.sin(base.heading.toDouble());
        double dx =  ch*fwd - sh*left, dy = sh*fwd + ch*left;
        return new Pose2d(base.position.x + dx, base.position.y + dy, base.heading.toDouble() + dThetaRad);
    }

    // helpers
    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }
    private static double clampWithKick(double v, double clamp, double minCmd){
        double out = clamp(v, -clamp, clamp);
        if (Math.abs(out) > 1e-6 && Math.abs(out) < minCmd) out = Math.copySign(minCmd, out);
        return out;
    }
    private static double wrap(double a){ while(a>Math.PI)a-=2*Math.PI; while(a<-Math.PI)a+=2*Math.PI; return a; }
}
