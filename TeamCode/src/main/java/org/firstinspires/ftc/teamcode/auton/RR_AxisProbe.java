package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="RR Axis Probe (force pose + left 24in)", group="debug")
public class RR_AxisProbe extends LinearOpMode {

    // FIELD (MeepMeep): X right, Y up   ->   RR: x forward, y left
    private static double angFieldToRR(double rad){ return rad - Math.PI/2.0; }
    private static Vector2d vecFieldToRR(double xF, double yF){ return new Vector2d(yF, -xF); }
    private static Pose2d poseFieldToRR(double xF, double yF, double degF){
        return new Pose2d(yF, -xF, angFieldToRR(Math.toRadians(degF)));
    }

    @Override public void runOpMode() {
        // Your MeepMeep start (-67.05, 26.72, -26.87°) -> RR
        Pose2d start = poseFieldToRR(-67.05, 26.72, -26.87);

        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        // Force the estimate repeatedly in INIT (in case anything resets it)
        while (opModeInInit()) {
            drive.localizer.setPose(start);
            Pose2d p = drive.localizer.getPose();
            telemetry.addData("FORCED Start (RR)", "(%.1f, %.1f, %.1f°)",
                    start.position.x, start.position.y, Math.toDegrees(start.heading.toDouble()));
            telemetry.addData("Pinpoint NOW",       "(%.1f, %.1f, %.1f°)",
                    p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
            telemetry.addLine("Place robot at this pose; heading ≈ -116.9°");
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        if (isStopRequested()) return;

        // Force again AFTER start in case your drive resets pose on start
        drive.localizer.setPose(start);

        // Ask for a PURE LEFT move of 24 in at the SAME heading
        // Start (x0,y0,h) -> Target (x0, y0 + 24, h). Tangent = -90° in RR
        double x0 = start.position.x, y0 = start.position.y, h = start.heading.toDouble();
        Vector2d tgt = new Vector2d(x0, y0 + 24);
        double tanLeft = -Math.PI/2.0;

        Action action = drive.actionBuilder(start)
                .splineToConstantHeading(tgt, tanLeft)
                .build();

        Actions.runBlocking(action);
    }
}
