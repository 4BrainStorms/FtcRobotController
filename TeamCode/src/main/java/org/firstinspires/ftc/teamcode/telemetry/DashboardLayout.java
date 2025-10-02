
package org.firstinspires.ftc.teamcode.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.LaunchZoneToggle;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

/**
 * Streams unified telemetry to FTC Dashboard for FTC Decode.
 */
public class DashboardLayout {

    private final FtcDashboard dashboard;
    private final DcMotor intakeMotor;
    private final DcMotor launcherLeft;
    private final DcMotor launcherRight;
    private final Limelight3A limelight;

    public DashboardLayout(DcMotor intakeMotor, DcMotor launcherLeft, DcMotor launcherRight, Limelight3A limelight) {
        this.dashboard = FtcDashboard.getInstance();
        this.intakeMotor = intakeMotor;
        this.launcherLeft = launcherLeft;
        this.launcherRight = launcherRight;
        this.limelight = limelight;
    }

    public void stream(Pose2d robotPose) {
        TelemetryPacket packet = new TelemetryPacket();

        // Pose
        packet.put("Pose X (in)", String.format("%.1f", robotPose.position.x));
        packet.put("Pose Y (in)", String.format("%.1f", robotPose.position.y));
        packet.put("Pose Heading (deg)", String.format("%.0f", Math.toDegrees(robotPose.heading.toDouble())));

        // Intake
        packet.put("Intake Power", String.format("%.2f", intakeMotor.getPower()));

        // Launcher
        packet.put("Launcher Left Power", String.format("%.2f", launcherLeft.getPower()));
        packet.put("Launcher Right Power", String.format("%.2f", launcherRight.getPower()));
        packet.put("Launch Zone", LaunchZoneToggle.useFrontZone ? "Front" : "Back");

        // AprilTag
        if (limelight != null) {
            LLResult res = limelight.getLatestResult();
            if (res != null && res.isValid()) {
                List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
                if (fids != null && !fids.isEmpty()) {
                    LLResultTypes.FiducialResult fid = fids.get(0);
                    packet.put("Tag ID", fid.getFiducialId());
                    packet.put("Tag Tx (deg)", String.format("%.2f", fid.getTargetXDegrees()));
                    packet.put("Tag Ty (deg)", String.format("%.2f", fid.getTargetYDegrees()));
                    packet.put("Tag Area", String.format("%.3f", fid.getTargetArea()));
                } else {
                    packet.put("Tag ID", "None");
                }
            } else {
                packet.put("Tag ID", "Invalid");
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }
}
