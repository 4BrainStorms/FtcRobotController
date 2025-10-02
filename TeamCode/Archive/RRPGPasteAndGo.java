package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Paths;

import java.util.List;

@Autonomous(name = "RRPathGen Paste And Go", group = "auton")
public class RRPGPasteAndGo extends LinearOpMode {

    // ====== PASTE YOUR RRPathGen EXPORT HERE (center-origin inches). Keep ".build();" ======
    private static final String RR_EXPORT =
            "TrajectorySequence trajectorytest = drive.trajectorySequenceBuilder(new Pose2d(23.69, -71.06, Math.toRadians(90.00)))\n" +
                    ".lineToLinearHeading(new Pose2d(23.69, -48.03, Math.toRadians(90.00)))\n" +
                    ".lineToLinearHeading(new Pose2d(-23.50, -47.84, Math.toRadians(179.77)))\n" +
                    ".build();\n";
    // =======================================================================================

    // Limelight config
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int PIPELINE_ALLTAGS   = 0;
    private static final int PIPELINE_GOAL_BLUE = 2; // your setup
    private static final int PIPELINE_GOAL_RED  = 3;
    private static final long MAX_STALENESS_MS  = 500;

    // Fallback (open-loop) gains when kS/kV/kA are zero
    private static final double K_POS = 0.04, K_YAW = 0.02;
    private static final double LIN_CLAMP = 0.60, YAW_CLAMP = 0.40;
    private static final double STOP_POS_IN = 1.5, STOP_YAW_RAD = Math.toRadians(2.0);

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    @Override
    public void runOpMode() {

        // Build path from the RRPathGen export:
        // Frame.CENTER because RRPathGen uses center-origin by default.
        // true => prepend your D1 start pose.
        Pose2d[] path = Paths.fromRRPathGen(RR_EXPORT, Paths.Frame.CENTER, /*startFromD1=*/true);
        Pose2d startPose = path[0];

        // Init drive at the computed start pose (D1 if startFromD1=true)
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Limelight3A ll = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        try { ll.setPollRateHz(100); } catch (Throwable ignored) {}
        ll.pipelineSwitch(PIPELINE_ALLTAGS);
        ll.start();

        // Choose alliance during INIT
        boolean lastL=false, lastR=false;
        while (opModeInInit()) {
            if (gamepad1.dpad_left && !lastL)  alliance = Alliance.RED;
            if (gamepad1.dpad_right && !lastR) alliance = Alliance.BLUE;
            lastL = gamepad1.dpad_left; lastR = gamepad1.dpad_right;

            LLResult res = ll.getLatestResult();
            telemetry.addData("Alliance", alliance);
            telemetry.addData("LL valid", res != null && res.isValid());
            telemetry.addData("stale (ms)", res==null? -1 : res.getStaleness());
            telemetry.addData("Start pose", "x=%.2f y=%.2f th=%.1f°",
                    startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble()));
            telemetry.addLine("D-Pad LEFT=RED, RIGHT=BLUE");
            telemetry.update();
            sleep(15);
        }

        waitForStart();
        if (isStopRequested()) return;

        // Help MT2 with yaw
        try {
            double yawDeg = Math.toDegrees(drive.localizer.getPose().heading.toDouble());
            ll.updateRobotOrientation(yawDeg);
        } catch (Throwable ignored) {}

        // Run the path (RR trajectories if FF tuned; else open-loop fallback)
        boolean ffTuned = !(MecanumDrive.PARAMS.kS == 0 && MecanumDrive.PARAMS.kV == 0 && MecanumDrive.PARAMS.kA == 0);

        if (ffTuned) {
            TrajectoryActionBuilder tab = drive.actionBuilder(startPose);
            for (int i = 1; i < path.length; i++) {
                Pose2d p = path[i];
                tab = tab.splineToLinearHeading(p, p.heading.toDouble());
            }
            Action runPath = tab.build();
            Actions.runBlocking(runPath);
        } else {
            // Open-loop fallback
            for (int i = 1; i < path.length; i++) {
                goToPoseOpenLoop(drive, path[i], 8000);
            }
        }

        // Optional: LL yaw align to goal
        ll.pipelineSwitch(alliance == Alliance.RED ? PIPELINE_GOAL_RED : PIPELINE_GOAL_BLUE);
        sleep(150);
        yawAlignToGoal(drive, ll, 2000);

        // stop
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        drive.updatePoseEstimate();
    }

    // ---------- helpers ----------
    private void goToPoseOpenLoop(MecanumDrive drive, Pose2d goal, long timeoutMs) {
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis()-t0 < timeoutMs) {
            Pose2d pose = drive.localizer.getPose();
            double dx = goal.position.x - pose.position.x;
            double dy = goal.position.y - pose.position.y;

            double c = Math.cos(pose.heading.toDouble());
            double s = Math.sin(pose.heading.toDouble());
            double errFwd  =  c*dx + s*dy;
            double errLeft = -s*dx + c*dy;
            double errYaw  = wrap(goal.heading.toDouble() - pose.heading.toDouble());

            double fwd  = clamp(K_POS*errFwd,  -LIN_CLAMP, LIN_CLAMP);
            double left = clamp(K_POS*errLeft, -LIN_CLAMP, LIN_CLAMP);
            double yaw  = clamp(K_YAW*errYaw,  -YAW_CLAMP, YAW_CLAMP);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, left), yaw));
            drive.updatePoseEstimate();

            if (Math.abs(errFwd) < STOP_POS_IN &&
                    Math.abs(errLeft) < STOP_POS_IN &&
                    Math.abs(errYaw) < STOP_YAW_RAD) break;

            sleep(10);
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        drive.updatePoseEstimate();
    }

    private void yawAlignToGoal(MecanumDrive drive, Limelight3A ll, long timeoutMs) {
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis()-t0 < timeoutMs) {
            LLResult res = ll.getLatestResult();
            if (res == null || !res.isValid() || res.getStaleness() > MAX_STALENESS_MS) break;
            List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
            if (fids == null || fids.isEmpty()) break;

            double txDeg = fids.get(0).getTargetXDegrees();
            double yawErr = Math.toRadians(txDeg);
            if (Math.abs(yawErr) < Math.toRadians(2.0)) break;

            double yawCmd = clamp(K_YAW*yawErr, -YAW_CLAMP, YAW_CLAMP);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), yawCmd));
            drive.updatePoseEstimate();
            sleep(10);
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        drive.updatePoseEstimate();
    }

    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }
    private static double wrap(double a){ while(a>Math.PI)a-=2*Math.PI; while(a<-Math.PI)a+=2*Math.PI; return a; }
}
