package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Pinpoint Sanity", group = "zz")
public class PinpointSanity extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Seed pose at field center (center-origin frame)
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Pinpoint sanity: push the robot by hand.");
        telemetry.addLine("Expect: forward→ +Y, left→ +X, CCW→ +heading.");
        telemetry.addLine("Press A to zero pose (0,0,0).");
        telemetry.update();

        // INIT loop
        while (opModeInInit()) {
            if (gamepad1.a) {
                drive.localizer.setPose(new Pose2d(0, 0, 0));
            }
            Pose2d p = drive.localizer.getPose();
            telemetry.addData("Pose (in)", "x=%.2f  y=%.2f", p.position.x, p.position.y);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(p.heading.toDouble()));
            telemetry.update();
            sleep(50);
        }

        waitForStart();
        if (isStopRequested()) return;

        // RUN loop — watch values while you move the bot
        while (opModeIsActive()) {
            Pose2d p = drive.localizer.getPose();
            telemetry.addData("Pose (in)", "x=%.2f  y=%.2f", p.position.x, p.position.y);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(p.heading.toDouble()));
            telemetry.update();
            sleep(50);
        }
    }
}
