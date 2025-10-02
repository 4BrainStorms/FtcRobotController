package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DriveBase {
    private MecanumDrive drive;

    public DriveBase(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new MecanumDrive(hardwareMap, startPose);
    }

    public MecanumDrive getDrive() {
        return drive;
    }

    public Action buildTrajectorySequence(Pose2d startPose) {
        return drive.actionBuilder(startPose)
            .splineToLinearHeading(new Pose2d(24, 0, 0), 0)
            .build();
    }
}