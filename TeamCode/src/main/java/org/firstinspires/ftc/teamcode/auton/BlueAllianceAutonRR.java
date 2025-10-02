package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * BLUE alliance auton using Limelight + Road Runner.
 * Starts in Square D with right wheels on line Y.
 */
@Autonomous(name="Auton: BLUE (LL+RR)", group="AA Auton")
public class BlueAllianceAutonRR extends LimelightAutonBaseRR {

    @Override protected boolean isBlueAlliance() { return true; }

    @Override protected Pose2d getStartPose() {
        // Field assumed origin at center, +x to audience right, +y away from driver.
        // Driver wall nominal x ≈ -60". "Line Y" at y=+24". Robot center is halfTrack to the LEFT of right wheels.
        double halfTrack = 7.0; // <-- measure yours (inches from center to right wheels)
        return new Pose2d(-60.0, +24.0 - halfTrack, 0.0);
    }

    @Override protected Action buildSequence(MecanumDrive drive, Pose2d startPose) {
        // TODO: replace with your RRPathGen-generated path (convert to actionBuilder calls)
        return RRPaths.examplePath(drive, startPose);
    }
}
