package org.firstinspires.ftc.teamcode.config;

/**
 * Provides calibrated motor speeds for launcher based on launch zone.
 */
public class LaunchCalibration {

    // Motor power values for each zone
    public static final double FRONT_ZONE_SPEED = 0.85;
    public static final double BACK_ZONE_SPEED = 0.95;

    /**
     * Returns the calibrated motor speed based on launch zone.
     * @param useFrontZone true if front zone is selected, false for back zone
     * @return motor power value
     */
    public static double getLaunchSpeed(boolean useFrontZone) {
        return useFrontZone ? FRONT_ZONE_SPEED : BACK_ZONE_SPEED;
    }
}