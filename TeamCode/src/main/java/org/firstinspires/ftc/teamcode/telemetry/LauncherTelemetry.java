package org.firstinspires.ftc.teamcode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.config.LaunchZoneToggle;
import org.firstinspires.ftc.teamcode.config.LaunchCalibration;

public class LauncherTelemetry {
    private final DcMotor launcherLeft;
    private final DcMotor launcherRight;
    private final FtcDashboard dashboard;

    public LauncherTelemetry(DcMotor launcherLeft, DcMotor launcherRight) {
        this.launcherLeft = launcherLeft;
        this.launcherRight = launcherRight;
        this.dashboard = FtcDashboard.getInstance();
    }

    public void stream(double launchSpeed, boolean isFrontZone) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Launcher Left Power", launcherLeft.getPower());
        packet.put("Launcher Right Power", launcherRight.getPower());
        packet.put("Launch Speed", launchSpeed);
        packet.put("Launch Zone", isFrontZone ? "Front" : "Back");
        dashboard.sendTelemetryPacket(packet);
    }
}