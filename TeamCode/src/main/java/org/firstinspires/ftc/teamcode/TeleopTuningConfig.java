package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TeleopTuningConfig {
    // Joystick -> command shaping
    public static double DEADZONE = 0.05;
    public static double INPUT_CURVE_EXP = 2.0;   // 2 = square, 3 = cubic

    // Slow mode (Right Trigger)
    // Final scale = 1.0 - (SLOW_MAX_REDUCTION * RT)
    public static double SLOW_MAX_REDUCTION = 0.70;

    // Heading hold
    public static double HEADING_HOLD_KP = 2.0;   // proportional gain
    public static double HEADING_HOLD_DEAD = 0.02; // |rx| below this => hold

    // Overlay
    public static double ARROW_SCALE_IN = 6.0;    // inches per full-stick
}
