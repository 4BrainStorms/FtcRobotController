package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Paths;

@Autonomous(name = "RRPathGen → Pinpoint (Simple)", group = "auton")
public class RRPG_PinpointSimple extends LinearOpMode {

    // ==== Paste your RRPathGen export here (center-origin inches; keep the trailing .build();) ====
    private static final String RR_EXPORT =
            "TrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))\n" +
                    "                        .forward(30)\n" +
                    "                        .turn(Math.toRadians(90))\n" +
                    "                        .forward(30)\n" +
                    "                        .turn(Math.toRadians(90))\n" +
                    "                        .forward(30)\n" +
                    "                        .turn(Math.toRadians(90))\n" +
                    "                        .forward(30)\n" +
                    "                        .turn(Math.toRadians(90))\n" +
                    "                        .build());\n;
    // ==============================================================================================

    // Basic holonomic P control (Pinpoint-only). Tweak if needed.
    private static final double K_POS = 0.045;                     // linear P
    private static final double K_YAW = 0.020;                     // angular P
    private static final double LIN_CLAMP = 0.65, YAW_CLAMP = 0.40;
    private static final double MIN_CMD = 0.07;                    // anti-stiction kick
    private static final double STOP_POS_IN = 1.25;                // arrive if within this (x & y)
    private static final double STOP_YAW_RAD = Math.toRadians(2.0);
    private static final long   SEG_TIMEOUT_MS = 9000;

    @Override
    public void runOpMode() {
        // Parse RRPathGen export into waypoints (field center-origin, inches). No D1, no anchoring.
        Pose2d[] path = Paths.fromRRPathGen(RR_EXPORT, Paths.Frame.CENTER, /*startFromD1=*/false);
        if (path == null || path.length < 1) {
            telemetry.addLine("RR_EXPORT parse failed or empty.");
            telemetry.update();
            return;
        }

        Pose2d start = path[0];
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        telemetry.addLine("RRPathGen path loaded (Simple).");
        telemetry.addData("Start pose", "x=%.2f y=%.2f θ=%.1f°",
                start.position.x, start.position.y, Math.toDegrees(start.heading.toDouble()));
        telemetry.addData("# waypoints (excluding start)", Math.max(0, path.length - 1));
        telemetry.addLine("Place robot at the exported start pose, then press ▶");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Walk each waypoint with a small Pinpoint-led P controller
        for (int i = 1; i < path.length && opModeIsActive(); i++) {
            goToPoseOpenLoop(drive, path[i], SEG_TIMEOUT_MS);
        }

        // stop
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        drive.updatePoseEstimate();
    }

    // --- Pinpoint-only holonomic P to a global pose ---
    private void goToPoseOpenLoop(MecanumDrive drive, Pose2d goal, long timeoutMs) {
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < timeoutMs) {
            Pose2d pose = drive.localizer.getPose();

            // error in field frame
            double dx = goal.position.x - pose.position.x;
            double dy = goal.position.y - pose.position.y;

            // rotate error into robot frame
            double ch = Math.cos(pose.heading.toDouble());
            double sh = Math.sin(pose.heading.toDouble());
            double errFwd  =  ch*dx + sh*dy;
            double errLeft = -sh*dx + ch*dy;
            double errYaw  = wrap(goal.heading.toDouble() - pose.heading.toDouble());

            // P control with clamps and anti-stiction kick
            double fwd  = clampWithKick(K_POS * errFwd,  LIN_CLAMP);
            double left = clampWithKick(K_POS * errLeft, LIN_CLAMP);
            double yaw  = clamp(K_YAW * errYaw, -YAW_CLAMP, YAW_CLAMP);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, left), yaw));
            drive.updatePoseEstimate();

            telemetry.addData("→ goal", "x=%.1f y=%.1f θ=%.1f°",
                    goal.position.x, goal.position.y, Math.toDegrees(goal.heading.toDouble()));
            telemetry.addData("pose", "x=%.1f y=%.1f θ=%.1f°",
                    pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("cmd",  "f=%.2f l=%.2f w=%.2f", fwd, left, yaw);
            telemetry.update();

            if (Math.abs(errFwd) < STOP_POS_IN &&
                    Math.abs(errLeft) < STOP_POS_IN &&
                    Math.abs(errYaw) < STOP_YAW_RAD) break;

            sleep(10);
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        drive.updatePoseEstimate();
        sleep(80);
    }

    // --- helpers ---
    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }
    private double clampWithKick(double v, double clamp){
        double out = clamp(v, -clamp, clamp);
        if (Math.abs(out) > 1e-6 && Math.abs(out) < MIN_CMD) out = Math.copySign(MIN_CMD, out);
        return out;
    }
    private static double wrap(double a){ while(a>Math.PI)a-=2*Math.PI; while(a<-Math.PI)a+=2*Math.PI; return a; }
}
