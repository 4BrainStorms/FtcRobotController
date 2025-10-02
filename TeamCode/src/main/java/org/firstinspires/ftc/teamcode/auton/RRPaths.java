package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Central place to hold Road Runner 1.0 Action sequences.
 * Replace the sample path with your RRPathGen-converted steps.
 */
public class RRPaths {

    /**
     * Example path: drive forward 24", strafe right 12", turn +90°.
     * Replace with your RRPathGen output using drive.actionBuilder(...).
     */
    public static Action examplePath(MecanumDrive drive, Pose2d start) {
        return drive.actionBuilder(start)
                .lineToX(start.position.x + 24.0)     // forward 24"
                .splineTo(new Vector2d(51.96, -32.68), Math.toRadians(60.08))
                .splineTo(new Vector2d(49.90, 45.41), Math.toRadians(51.36))
                .build();
    }
}
