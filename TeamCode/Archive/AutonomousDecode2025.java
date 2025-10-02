package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.ArtifactVisionBase;
import org.firstinspires.ftc.teamcode.subsystems.IntakeBase;
import org.firstinspires.ftc.teamcode.subsystems.LaunchBase;
import org.firstinspires.ftc.teamcode.subsystems.LimelightBase;
import org.firstinspires.ftc.teamcode.config.AllianceTagConfig;
import org.firstinspires.ftc.teamcode.config.LaunchCalibration;
import org.firstinspires.ftc.teamcode.config.LaunchZoneToggle;
import org.firstinspires.ftc.teamcode.config.RobotInitConfig;

import java.util.List;

/**
 * AutonomousDecode2025.java
 * FTC Decode autonomous logic using dual camera vision and machine state architecture.
 */
public class AutonomousDecode2025 extends LimelightAutonBaseRR {

    private ArtifactVisionBase artifactVision;
    private LimelightBase limelightBase;
    private IntakeBase intakeBase;
    private LaunchBase launchBase;

    @Override
    protected boolean isBlueAlliance() {
        return RobotInitConfig.IS_BLUE_ALLIANCE;
    }

    @Override
    protected Pose2d getStartPose() {
        return RobotInitConfig.START_POSE;
    }

    @Override
    protected Action buildSequence(MecanumDrive drive, Pose2d startPose) {
        HardwareMap hw = hardwareMap;

        // Initialize subsystems
        artifactVision = new ArtifactVisionBase(hw); // Stubbed
        limelightBase = new LimelightBase(hw, "limelight");
        intakeBase = new IntakeBase(hw);
        launchBase = new LaunchBase(hw);

        // Skip vision and use fallback pose
        Pose2d targetPose = AllianceTagConfig.getFallbackPose(isBlueAlliance());

        // Build trajectory to fallback
        Action toArtifact = drive.actionBuilder(startPose)
                .strafeTo(targetPose.position)
                .build();

        // Intake action
        Action intakeAction = (packet) -> {
            intakeBase.intakeOn();
            return false;
        };

        // Align and launch using AprilTag
        Action alignAndLaunch = (packet) -> {
            List<FiducialResult> tags = limelightBase.getFiducials();
            if (launchBase.alignToTag(tags)) {
                launchBase.launch(LaunchCalibration.getLaunchSpeed(LaunchZoneToggle.useFrontZone));
            }
            return false;
        };

        // Full sequence
        return new SequentialAction(
                toArtifact,
                intakeAction,
                alignAndLaunch
        );
    }
}