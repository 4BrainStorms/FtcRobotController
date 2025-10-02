package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.config.LaunchZoneToggle;
import org.firstinspires.ftc.teamcode.config.LaunchCalibration;
import org.firstinspires.ftc.teamcode.telemetry.LauncherTelemetry;

public class AutonomousLogicEnhancer {
    private final DcMotor launcherLeft;
    private final DcMotor launcherRight;
    private final LauncherTelemetry telemetry;

    public AutonomousLogicEnhancer(DcMotor launcherLeft, DcMotor launcherRight) {
        this.launcherLeft = launcherLeft;
        this.launcherRight = launcherRight;
        this.telemetry = new LauncherTelemetry(launcherLeft, launcherRight);
    }

    public void calibrateAndLaunch() {
        boolean isFrontZone = LaunchZoneToggle.useFrontZone;
        double speed = LaunchCalibration.getLaunchSpeed(isFrontZone);

        launcherLeft.setPower(speed);
        launcherRight.setPower(-speed); // Opposite direction

        telemetry.stream(speed, isFrontZone);
    }

    public void stopLaunchers() {
        launcherLeft.setPower(0);
        launcherRight.setPower(0);
    }
}