package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.util.StartPoses;

/** Holds paths exported from RRPathGen and converts center-origin -> corner-origin (inches). */
public final class PathLibraryRRPG {
    private PathLibraryRRPG() {}

    private static final double HALF_FIELD_IN = 72.0; // 144"/2

    /** Convert RRPathGen center-origin inches to our corner-origin inches. */
    private static Pose2d toCorner(Pose2d centerOriginPose) {
        return new Pose2d(
                centerOriginPose.position.x + HALF_FIELD_IN,
                centerOriginPose.position.y + HALF_FIELD_IN,
                centerOriginPose.heading.toDouble()
        );
    }

    // -------------------------------------------------------------------------
    // NEW EXPORT YOU PROVIDED (center-origin):
    // TrajectorySequence trajectorytest = drive.trajectorySequenceBuilder(
    //     new Pose2d(23.50, -47.09, Math.toRadians(90.00)))
    //   .splineTo(new Vector2d(11.33,  9.83), Math.toRadians(74.88))
    //   .splineToLinearHeading(new Pose2d(38.67, 33.61, Math.toRadians(79.36)), Math.toRadians(79.36))
    //   .build();
    //
    // Waypoints we’ll use:
    //   p1: (11.33,  9.83, 74.88°)
    //   p2: (38.67, 33.61, 79.36°)
    // -------------------------------------------------------------------------

    /** A) Start at your D1 placement, then go through the new waypoints. */
    public static Pose2d[] TRAJECTORYTEST_FROM_D1() {
        Pose2d p1Corner = toCorner(new Pose2d(11.33,  9.83, Math.toRadians(74.88)));
        Pose2d p2Corner = toCorner(new Pose2d(38.67, 33.61, Math.toRadians(79.36)));

        return new Pose2d[] {
                StartPoses.START_D1,  // your real field start
                p1Corner,
                p2Corner
        };
    }

    /** B) Run the path exactly as authored (robot must be placed at this converted start). */
    public static Pose2d[] TRAJECTORYTEST_AS_AUTHORED() {
        Pose2d startCorner = toCorner(new Pose2d(23.50, -47.09, Math.toRadians(90.00)));
        Pose2d p1Corner    = toCorner(new Pose2d(11.33,  9.83,  Math.toRadians(74.88)));
        Pose2d p2Corner    = toCorner(new Pose2d(38.67, 33.61, Math.toRadians(79.36)));

        return new Pose2d[] { startCorner, p1Corner, p2Corner };
    }
}
