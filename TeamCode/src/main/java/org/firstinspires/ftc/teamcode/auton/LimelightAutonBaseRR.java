package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

/**
 * Base class for Limelight-augmented Road Runner autonomous.
 * Compatible with the FTC SDK Limelight3A API:
 * - Use result.isValid() to check freshness
 * - Use result.getFiducialResults() for AprilTag list (no getTargetCount())
 * - Use FiducialResult.getTargetXDegrees()/getTargetYDegrees()/getTargetArea()
 */
public abstract class LimelightAutonBaseRR extends LinearOpMode {

    // ---- USER SETTINGS ----
    public static String LIMELIGHT_NAME = "limelight";

    // Pipelines you set up
    public static final int PIPE_ALL_TAGS     = 0;
    public static final int PIPE_OBELISK_ONLY = 1;
    public static final int PIPE_GOAL_BLUE_20 = 2;
    public static final int PIPE_GOAL_RED_24  = 3;

    public static boolean SELECT_PIPELINE_ON_INIT = true;

    // ---- RUNTIME ----
    protected FtcDashboard dashboard;
    protected Limelight3A limelight;
    protected MecanumDrive drive;

    // subclass must fill these in
    protected abstract boolean isBlueAlliance();
    protected abstract Pose2d getStartPose();
    protected abstract Action buildSequence(MecanumDrive drive, Pose2d startPose);

    protected int selectInitPipeline() {
        if (!SELECT_PIPELINE_ON_INIT) return -1;
        return isBlueAlliance() ? PIPE_GOAL_BLUE_20 : PIPE_GOAL_RED_24;
    }

    @Override
    public final void runOpMode() {
        dashboard = FtcDashboard.getInstance();

        drive = new MecanumDrive(hardwareMap, getStartPose());
        limelight = getLimelight(hardwareMap, LIMELIGHT_NAME);

        if (limelight != null) {
            try { limelight.setPollRateHz(100); } catch (Throwable ignored) {}
            try { limelight.start(); } catch (Throwable ignored) {}
        }

        int pipeline = selectInitPipeline();
        if (limelight != null && pipeline >= 0) {
            try { limelight.pipelineSwitch(pipeline); } catch (Throwable ignored) {}
        }

        telemetry.addLine(isBlueAlliance() ? "Alliance: BLUE" : "Alliance: RED");
        telemetry.addData("Start Pose", fmtPose(getStartPose()));
        telemetry.addData("LL Online", (limelight != null));
        telemetry.addData("Init Pipeline", pipeline);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Pose2d start = getStartPose();
        Action seq = buildSequence(drive, start);

        Actions.runBlocking(new Action() {
            @Override public boolean run(TelemetryPacket p) {
                streamLimelightTelemetry();
                return seq.run(p);
            }
        });

        drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                new com.acmerobotics.roadrunner.Vector2d(0,0), 0));
    }

    private Limelight3A getLimelight(HardwareMap hw, String name) {
        try { return hw.get(Limelight3A.class, name); }
        catch (Throwable t) {
            telemetry.addLine("Limelight not found: " + name);
            return null;
        }
    }

    /** Stream a few key LL values to DS + Dashboard (using supported API). */
    protected void streamLimelightTelemetry() {
        if (limelight == null) return;
        LLResult res = limelight.getLatestResult();
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Alliance", isBlueAlliance() ? "BLUE" : "RED");

        if (res != null && res.isValid()) {
            List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
            int count = (fids != null) ? fids.size() : 0;
            packet.put("LL Fiducials", count);
            telemetry.addData("LL Fiducials", count);

            if (count > 0) {
                LLResultTypes.FiducialResult fid = fids.get(0);
                packet.put("TagID", fid.getFiducialId());
                packet.put("Tx(deg)", fid.getTargetXDegrees());
                packet.put("Ty(deg)", fid.getTargetYDegrees());
                packet.put("Area", fid.getTargetArea());
                telemetry.addData("TagID", fid.getFiducialId());
                telemetry.addData("Tx/Ty", "%.2f / %.2f", fid.getTargetXDegrees(), fid.getTargetYDegrees());
                telemetry.addData("Area", "%.3f", fid.getTargetArea());
            }
        } else {
            packet.put("LL Fiducials", 0);
            telemetry.addData("LL Fiducials", 0);
        }

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

    private static String fmtPose(Pose2d p) {
        return String.format("(x=%.1f, y=%.1f, θ=%.0f°)", p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
    }
}
