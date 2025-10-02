package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Road Runner Localizer powered by goBILDA Pinpoint.
 *
 * Coordinate frames:
 *   - Road Runner:  x = forward,  y = left,  heading CCW+ (rad), inches
 *   - Pinpoint    :  X = forward,  Y = left,  heading CCW+ (rad), mm
 *
 * Mapping used here (DIRECT):
 *   RR.x <-> Pinpoint.X
 *   RR.y <-> Pinpoint.Y
 */
public final class PinpointLocalizer implements Localizer {
    private static final double MM_PER_IN = 25.4;

    public final GoBildaPinpointDriver driver;

    // --- Adjust to YOUR robot: pod offsets relative to the ROBOT TRACKING POINT (usually center), in Pinpoint frame.
    // Pinpoint offset sign convention:
    //   X_offset: +forward / -back
    //   Y_offset: +left    / -right
    public static final double X_OFFSET_MM = -163.0;  // strafe pod is 163 mm BEHIND  center -> -163 along forward axis
    public static final double Y_OFFSET_MM = -25.0;   // forward pod is 25 mm  to the RIGHT -> -25 along left axis

    private Pose2d pose;

    public PinpointLocalizer(HardwareMap hw, Pose2d startPose) {
        driver = hw.get(GoBildaPinpointDriver.class, "pinpoint"); // rename if your config uses a different name

        // Set pod model / resolution (or use setEncoderResolution(mmPerTick))
        driver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Geometry (in Pinpoint frame units)
        driver.setOffsets(X_OFFSET_MM, Y_OFFSET_MM, DistanceUnit.MM);

        // Encoder directions: start with FORWARD/FO RWARD; flip X later only if lateral sign is mirrored
        driver.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,  // X encoder (forward)
                GoBildaPinpointDriver.EncoderDirection.FORWARD   // Y encoder (left)
        );

        // Reset internal accumulators and IMU fusion
        driver.resetPosAndIMU();

        // Seed BOTH heading and XY to match RR start pose
        setPose(startPose);
    }

    @Override
    public void setPose(Pose2d p) {
        this.pose = p;

        // Seed heading first
        driver.setHeading(p.heading.toDouble(), AngleUnit.RADIANS);

        // Seed XY (RR inches -> Pinpoint mm), DIRECT mapping:
        double pinX_mm = p.position.x * MM_PER_IN; // RR forward -> Pinpoint X
        double pinY_mm = p.position.y * MM_PER_IN; // RR left    -> Pinpoint Y

        driver.setPosition(new Pose2D(
                DistanceUnit.MM,      // position units
                pinX_mm,              // X (mm)
                pinY_mm,              // Y (mm)
                AngleUnit.RADIANS,    // heading units
                p.heading.toDouble()  // heading
        ));
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        // One I2C read for all fields
        driver.update();

        // Position: Pinpoint mm -> RR inches (DIRECT mapping)
        Pose2D pp = driver.getPosition();
        double rrX_in = pp.getX(DistanceUnit.MM) / MM_PER_IN; // forward
        double rrY_in = pp.getY(DistanceUnit.MM) / MM_PER_IN; // left
        double hRad   = pp.getHeading(AngleUnit.RADIANS);

        // Velocities: mm/s -> in/s (DIRECT mapping)
        double vFwd_inps  = driver.getVelX(DistanceUnit.MM) / MM_PER_IN; // forward
        double vLeft_inps = driver.getVelY(DistanceUnit.MM) / MM_PER_IN; // left
        double wRadPerS   = driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        // Update RR pose and return RR velocity
        pose = new Pose2d(rrX_in, rrY_in, hRad);
        return new PoseVelocity2d(new Vector2d(vFwd_inps, vLeft_inps), wRadPerS);
    }
}
