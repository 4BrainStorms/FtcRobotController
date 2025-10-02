package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Field→RR Quick (one path)", group="auton")
public class RRPG_FieldToRR_Quick extends LinearOpMode {

    // FIELD (X right, Y up, 0°=+X) -> RR (x forward, y left, 0°=+x)
    private static double angFieldToRR(double rad){ return rad - Math.PI/2.0; }
    private static Vector2d vecFieldToRR(double xF, double yF){ return new Vector2d(yF, -xF); }
    private static Pose2d poseFieldToRR(double xF, double yF, double degF){
        return new Pose2d(yF, -xF, angFieldToRR(Math.toRadians(degF)));
    }

    @Override public void runOpMode() {
        // MeepMeep start: (-67.05, 26.72, -26.87°)
        Pose2d start = poseFieldToRR(-67.05, 26.72, -26.87);

        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        // Seed Pinpoint/RR pose in INIT
        drive.localizer.setPose(start);

        // Build path (convert waypoint + tangent)
        Vector2d p1  = vecFieldToRR(-22.79, 4.73);
        double  tan1 = angFieldToRR(Math.toRadians(-26.41));

        Action action = drive.actionBuilder(start)
                .splineTo(p1, tan1)
                .build();

        while (opModeInInit()){
            if (gamepad1.x) drive.localizer.setPose(start);
            Pose2d pp = drive.localizer.getPose();
            telemetry.addData("Start(RR)", "(%.1f, %.1f, %.1f°)",
                    start.position.x, start.position.y, Math.toDegrees(start.heading.toDouble()));
            telemetry.addData("Pinpoint",  "(%.1f, %.1f, %.1f°)",
                    pp.position.x, pp.position.y, Math.toDegrees(pp.heading.toDouble()));
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        drive.localizer.setPose(start);   // ⚠️ re-seed once right after start

        Actions.runBlocking(action);
    }
}
