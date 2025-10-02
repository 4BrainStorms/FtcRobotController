package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Pinpoint Heading Check", group = "debug")
public class PinpointHeadingCheck extends LinearOpMode {
    @Override public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        // Show live heading BEFORE start
        while (opModeInInit()) {
            drive.updatePoseEstimate();  // <-- important in INIT
            Pose2d p = drive.localizer.getPose();
            telemetry.addData("Pose (in)", "x=%.2f  y=%.2f", p.position.x, p.position.y);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(p.heading.toDouble()));
            telemetry.addLine("Point robot along +X. Expect ≈ 0°. CCW turn should increase.");
            telemetry.update();
            sleep(20);
        }

        waitForStart();
    }
}
