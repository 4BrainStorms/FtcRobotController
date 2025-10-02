package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.roadrunner.Pose2d;
import java.util.HashMap;
import java.util.Map;

/**
 * Maps AprilTag IDs to field poses based on alliance color.
 */
public class AllianceTagConfig {

    public static Map<Integer, Pose2d> getTagPoseMap(boolean isBlueAlliance) {
        Map<Integer, Pose2d> tagPoseMap = new HashMap<>();

        if (isBlueAlliance) {
            // Blue alliance tag mappings
            tagPoseMap.put(1, new Pose2d(12, 60, Math.toRadians(180)));
            tagPoseMap.put(2, new Pose2d(36, 60, Math.toRadians(180)));
            tagPoseMap.put(3, new Pose2d(60, 60, Math.toRadians(180)));
            tagPoseMap.put(20, new Pose2d(36, 12, Math.toRadians(0))); // Blue launch zone
        } else {
            // Red alliance tag mappings
            tagPoseMap.put(4, new Pose2d(12, -60, Math.toRadians(0)));
            tagPoseMap.put(5, new Pose2d(36, -60, Math.toRadians(0)));
            tagPoseMap.put(6, new Pose2d(60, -60, Math.toRadians(0)));
            tagPoseMap.put(24, new Pose2d(36, -12, Math.toRadians(180))); // Red launch zone
        }

        return tagPoseMap;
    }

    public static Pose2d getArtifactPose(boolean isBlueAlliance, String label) {
        if (isBlueAlliance) {
            switch (label) {
                case "green": return new Pose2d(12, 48, Math.toRadians(180));
                case "purple": return new Pose2d(36, 48, Math.toRadians(180));
                case "robot": return new Pose2d(60, 48, Math.toRadians(180));
            }
        } else {
            switch (label) {
                case "green": return new Pose2d(12, -48, Math.toRadians(0));
                case "purple": return new Pose2d(36, -48, Math.toRadians(0));
                case "robot": return new Pose2d(60, -48, Math.toRadians(0));
            }
        }
        return getFallbackPose(isBlueAlliance);
    }

    public static Pose2d getFallbackPose(boolean isBlueAlliance) {
        return isBlueAlliance
                ? new Pose2d(36, 60, Math.toRadians(180))
                : new Pose2d(36, -60, Math.toRadians(0));
}}

