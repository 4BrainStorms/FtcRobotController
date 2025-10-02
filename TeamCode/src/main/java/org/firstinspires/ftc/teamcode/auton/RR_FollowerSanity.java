package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="RR Follower Sanity (0,0,0 → left 24\")", group="debug")
public class RR_FollowerSanity extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(0,0,0);   // heading 0 = forward
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        // Move left 24" at constant heading 0; path tangent for “left” is +π/2
        Vector2d tgt = new Vector2d(0, 24);
        double tanLeft = Math.PI / 2.0;

        Action action = drive.actionBuilder(start)
                .splineToConstantHeading(tgt, tanLeft)
                .build();

        // INIT: keep Pinpoint seeded; press X to snap again
        while (opModeInInit()) {
            if (gamepad1.x) drive.localizer.setPose(start);
            Pose2d p = drive.localizer.getPose();
            telemetry.addData("Start",    "(%.1f, %.1f, %.1f°)", start.position.x, start.position.y, Math.toDegrees(start.heading.toDouble()));
            telemetry.addData("Pinpoint", "(%.1f, %.1f, %.1f°)", p.position.x,     p.position.y,     Math.toDegrees(p.heading.toDouble()));
            telemetry.addLine("Place robot at (0,0,0). Press X to re-seed.");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Seed once more after start to avoid any reset quirks
        drive.localizer.setPose(start);
        Actions.runBlocking(action);
    }
}
