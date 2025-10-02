package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * Declares alliance and starting pose for autonomous.
 */
public class RobotInitConfig {

    // Set alliance color
    public static final boolean IS_BLUE_ALLIANCE = true;

    // Set starting pose (inches)
    public static final Pose2d START_POSE = IS_BLUE_ALLIANCE
        ? new Pose2d(12, 60, Math.toRadians(180))
        : new Pose2d(12, -60, Math.toRadians(0));
}