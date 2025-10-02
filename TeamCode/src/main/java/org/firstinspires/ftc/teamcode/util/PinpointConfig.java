// PinpointConfig.java  (put in org.firstinspires.ftc.teamcode.util)
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public final class PinpointConfig {
    private PinpointConfig() {}

    // TODO: put yours here (mm). Signs: +X = left of center, +Y = forward of center.
    public static double X_OFFSET_MM = -25.0;   // example
    public static double Y_OFFSET_MM = -163.0;  // example
    public static double YAW_SCALAR  = 1.0;    // tune later if needed

    public static GoBildaPinpointDriver init(HardwareMap hw, String name) {
        GoBildaPinpointDriver d = hw.get(GoBildaPinpointDriver.class, name); // e.g. "pinpoint"
        d.setOffsets(X_OFFSET_MM, Y_OFFSET_MM, DistanceUnit.MM);
        d.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        d.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        d.setYawScalar(YAW_SCALAR);
        d.resetPosAndIMU();    // call again at auton start if you prefer
        return d;
    }
}
