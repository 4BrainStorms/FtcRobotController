package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Autonomous(name = "RR_OdometryTest", group = "auton")
public class RR_OdometryTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-60.35, 35.98, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(-36.51, 35.80, Math.toRadians(-0.42)))
                        .lineToLinearHeading(new Pose2d(-37.00, -24.00, Math.toRadians(269.79)))
                        .build()
        );
    }
}
