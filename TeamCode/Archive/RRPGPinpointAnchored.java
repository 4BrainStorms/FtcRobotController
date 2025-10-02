package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Paths;
import org.firstinspires.ftc.teamcode.util.StartPoses;

@Autonomous(name = "RRPathGen → Pinpoint (Anchored)", group = "auton")
public class RRPGPinpointAnchored extends LinearOpMode {

    // ==== Paste your RRPathGen export (center-origin inches; keep the trailing .build();) ====
    private static final String RR_EXPORT =
            "TrajectorySequence trajectorytest = drive.trajectorySequenceBuilder(new Pose2d(0.36, 0.66, Math.toRadians(90.00)))\n" +
                    ".lineToLinearHeading(new Pose2d(23.95, 0.00, Math.toRadians(-1.00)))\n" +
                    ".build();\n";
    // ========================================================================================

    // If true: we ignore the export’s start pose and prepend your D1 pose, then anchor the whole path
    // to whatever Pinpoint says your current pose is at INIT (so you can always start on the D1 line).
    // If false: the robot must be placed at the export’s start.
    private static final boolean START_FROM_D1 = true;
    private static final boolean ANCHOR_TO_CURRENT = true;

    // Pinpoint-led holonomic P
    private static final double K_POS = 0.045;
    private static final double K_YAW = 0.020;
    private static final double LIN_CLAMP = 0.65, YAW_CLAMP = 0.40;
    private static final double MIN_CMD = 0.07;                 // small kick to overcome stiction
    private static final double STOP_POS_IN = 1.25;
    private static final double STOP_YAW_RAD = Math.toRadians(2.0);
    private static final long   SEG_TIMEOUT_MS = 9000;

    @Override
    public void runOpMode() {
        // Parse RR export → CORNER frame poses
        Pose2d[] rawPath = Paths.fromRRPathGen(RR_EXPORT, Paths.Frame.CENTER, START_FROM_D1);

        // Bring up drive with a reasonable initial pose (D1 or export start)
        Pose2d initPose = rawPath[0];
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        // Allow anchoring the entire path to the robot’s *actual* Pinpoint pose at INIT
        Pose2d[] path = rawPath;

        telemetry.addLine("INIT: Press A to re-anchor path to current pose");
        while (opModeInInit()) {
            Pose2d pp = drive.localizer.getPose();
            telemetry.addData("Pinpoint pose", "x=%.2f y=%.2f θ=%.1f°",
                    pp.position.x, pp.position.y, Math.toDegrees(pp.heading.toDouble()));
            telemetry.addData("Export start", "x=%.2f y=%.2f θ=%.1f°",
                    rawPath[0].position.x, rawPath[0].position.y, Math.toDegrees(rawPath[0].heading.toDouble()));
            telemetry.addData("# waypoints", Math.max(0, rawPath.length - 1));
            if (ANCHOR_TO_CURRENT && gamepad1.a) {
                path = anchorPath(rawPath, pp);    // shift+rotate whole path so start == current
                telemetry.addLine("Path ANCHORED to current pose ✔");
            } else {
                telemetry.addLine(ANCHOR_TO_CURRENT ? "Press A to anchor" : "Anchoring disabled");
            }
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        if (isStopRequested()) return;

        // Walk the waypoints with Pinpoint-only P control
        for (int i = 1; i < path.length && opModeIsActive(); i++) {
            goToPoseOpenLoop(drive, path[i], SEG_TIMEOUT_MS);
        }

        // stop
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        drive.updatePoseEstimate();
    }

    // Shift+rotate entire path so that path[0] coincides with "anchor" pose
    private Pose2d[] anchorPath(Pose2d[] in, Pose2d anchor) {
        Pose2d s = in[0];
        double dth = anchor.heading.toDouble() - s.heading.toDouble();
        double c = Math.cos(dth), sN = Math.sin(dth);
        Pose2d[] out = new Pose2d[in.length];
        for (int i = 0; i < in.length; i++) {
            double dx = in[i].position.x - s.position.x;
            double dy = in[i].position.y - s.position.y;
            double rx =  c*dx - sN*dy;
            double ry =  sN*dx + c*dy;
            double nx = anchor.position.x + rx;
            double ny = anchor.position.y + ry;
            double nh = in[i].heading.toDouble() + dth;
            out[i] = new Pose2d(nx, ny, nh);
        }
        return out;
    }

    // Pinpoint-only holonomic P to a global pose
    private void goToPoseOpenLoop(MecanumDrive drive, Pose2d goal, long timeoutMs) {
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < timeoutMs) {
            Pose2d pose = drive.localizer.getPose();

            // global error
            double dx = goal.position.x - pose.position.x;
            double dy = goal.position.y - pose.position.y;

            // → robot frame
            double c = Math.cos(pose.heading.toDouble());
            double s = Math.sin(pose.heading.toDouble());
            double errFwd  =  c*dx + s*dy;
            double errLeft = -s*dx + c*dy;
            double errYaw  = wrap(goal.heading.toDouble() - pose.heading.toDouble());

            // P → power
            double fwd  = clampWithKick(K_POS*errFwd,  LIN_CLAMP);
            double left = clampWithKick(K_POS*errLeft, LIN_CLAMP);
            double yaw  = clamp(K_YAW*errYaw, -YAW_CLAMP, YAW_CLAMP);

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
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        drive.updatePoseEstimate();
        sleep(80);
    }

    // Helpers
    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }
    private double clampWithKick(double v, double clamp){
        double out = clamp(v, -clamp, clamp);
        if (Math.abs(out) > 1e-6 && Math.abs(out) < MIN_CMD) out = Math.copySign(MIN_CMD, out);
        return out;
    }
    private static double wrap(double a){ while(a>Math.PI)a-=2*Math.PI; while(a<-Math.PI)a+=2*Math.PI; return a; }
}
