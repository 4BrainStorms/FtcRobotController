package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;

/** Start poses in CENTER-ORIGIN field coords (−72..+72 in). */
public final class StartPoses {
    private StartPoses() {}

    public static final double TILE_IN      = 24.0;

    // Robot geometry (wheel-center to wheel-center)
    public static final double TRACK_IN     = 12.79; // 32.5 cm
    public static final double WHEELBASE_IN = 13.18; // 33.5 cm

    // ----- Field frame helpers -----
    public static final double FIELD_HALF_IN = 72.0;     // corner→center shift

    // Grid lines (CENTER origin)
    // D–E vertical grid line (x = 4 tiles = 96") → 96 - 72 = +24 (center frame)
    public static final double DE_X_CENTER = 4 * TILE_IN - FIELD_HALF_IN;  // +24
    // "1" horizontal line (y = 1 tile = 24") → 24 - 72 = −48 (center frame)
    public static final double ROW1_Y_CENTER = 1 * TILE_IN - FIELD_HALF_IN; // −48

    /** D1 start in CENTER origin.
     *  Place robot so its wheel-center is one half TRACK/WHEELBASE inside the lines. */
    public static final Pose2d START_D1_CENTER = new Pose2d(
            DE_X_CENTER  - TRACK_IN / 2.0,     // 24  - 6.395 = 17.605
            ROW1_Y_CENTER - WHEELBASE_IN / 2.0, // -48 - 6.590 = -54.590
            0.0 // radians (0 = +X/right)
    );

    // (Optional) Keep the old CORNER-origin value for comparison/telemetry, if needed.
    /** @deprecated Corner-origin version, kept for reference. Prefer START_D1_CENTER. */
    @Deprecated
    public static final Pose2d START_D1_CORNER = new Pose2d(
            4 * TILE_IN - TRACK_IN / 2.0,       // 96  - 6.395 = 89.605
            1 * TILE_IN - WHEELBASE_IN / 2.0,   // 24  - 6.590 = 17.410
            0.0
    );
}
