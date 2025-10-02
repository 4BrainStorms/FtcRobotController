package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Paths;

@Autonomous(name = "RRPathGen → Actions (Paste & Run)", group = "auton")
public class RRPG_ActionsRun extends LinearOpMode {

    // === Paste your RRPathGen export exactly as copied (center-origin inches; keep .build();) ===
    private static final String RR_EXPORT =
            "TrajectorySequence trajectorytest = drive.trajectorySequenceBuilder(new Pose2d(-71.42, 71.27, Math.toRadians(90.00)))\n" +
                    ".lineToConstantHeading(new Vector2d(-48.27, 48.70))\n" +
                    ".build();\n";
    // ============================================================================================

    // Turn at every waypoint or only at the very end?
    private static final boolean TURN_AT_EACH_WAYPOINT = false;

    @Override
    public void runOpMode() {
        // Parse RR export into Pose2d[] in the CENTER frame; don't auto-D1/anchor
        Pose2d[] pts = Paths.fromRRPathGen(RR_EXPORT, Paths.Frame.CENTER, /*startFromD1=*/false);
        if (pts == null || pts.length < 2) {
            telemetry.addLine("RR_EXPORT parse failed or needs at least 2 poses.");
            telemetry.update();
            return;
        }

        // Bring up drive seeded at the export start; this anchors Pinpoint to start pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, pts[0]);

        // (Recommended) gentle starting gains for follower; tune in Dashboard if needed
        MecanumDrive.PARAMS.axialGain   = 2.0;   // translational P (forward/back)
        MecanumDrive.PARAMS.lateralGain = 2.0;   // translational P (strafe)
        MecanumDrive.PARAMS.headingGain = 8.0;   // heading P
        // leave velocity gains at 0.0 to start (can add later)
        // profile limits (optional):
        // MecanumDrive.PARAMS.maxWheelVel = 45;   // in/s
        // MecanumDrive.PARAMS.maxAngVel   = Math.PI; // rad/s

        // Build a smooth action: spline between waypoints, with optional in-place turns
        TrajectoryActionBuilder b = drive.actionBuilder(pts[0]);
        for (int i = 1; i < pts.length; i++) {
            Pose2d prev = pts[i - 1];
            Pose2d cur  = pts[i];

            // spline direction toward the next point
            double tan = Math.atan2(cur.position.y - prev.position.y, cur.position.x - prev.position.x);
            b = b.splineTo(new Vector2d(cur.position.x, cur.position.y), tan);

            if (TURN_AT_EACH_WAYPOINT) {
                double dth = wrap(cur.heading.toDouble() - prev.heading.toDouble());
                if (Math.abs(dth) > Math.toRadians(1.0)) {
                    b = b.turn(dth);  // clean in-place turn to requested heading
                }
            }
        }
        // If we didn’t turn at each point, turn once at the very end to match final heading.
        if (!TURN_AT_EACH_WAYPOINT) {
            double dthEnd = wrap(pts[pts.length - 1].heading.toDouble() - pts[pts.length - 2].heading.toDouble());
            if (Math.abs(dthEnd) > Math.toRadians(1.0)) {
                b = b.turn(dthEnd);
            }
        }

        Action action = b.build();

        telemetry.addLine("RRPathGen path ready (Actions)");
        telemetry.addData("Start", "x=%.2f y=%.2f θ=%.1f°",
                pts[0].position.x, pts[0].position.y, Math.toDegrees(pts[0].heading.toDouble()));
        telemetry.addData("End",   "x=%.2f y=%.2f θ=%.1f°",
                pts[pts.length-1].position.x, pts[pts.length-1].position.y,
                Math.toDegrees(pts[pts.length-1].heading.toDouble()));
        telemetry.addData("# waypoints", pts.length);
        telemetry.addLine("Place robot at the RR export START pose & heading.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Follow the full path + final heading with the smooth follower
        Actions.runBlocking(action);

        // done (motors are set to 0 at the end by the follower)
    }

    private static double wrap(double a){
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
}
