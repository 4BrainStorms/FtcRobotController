package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="RR Raw Drive Probe", group="debug")
public class RR_RawDriveProbe extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        // Use a real start pose to avoid NPEs
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Step 1: PURE LEFT  (y=+0.5) for 2s");
        telemetry.addLine("Step 2: PURE FWD   (x=+0.5) for 2s");
        telemetry.addLine("Step 3: ROTATE CCW (w=+0.8) for 1.5s");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Step 1: y+ (left)
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, +0.5), 0.0));
        sleep(2000);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        sleep(700);

        // Step 2: x+ (forward)
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(+0.5, 0.0), 0.0));
        sleep(2000);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        sleep(700);

        // Step 3: ω+ (CCW)
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), +0.8));
        sleep(1500);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
    }
}
