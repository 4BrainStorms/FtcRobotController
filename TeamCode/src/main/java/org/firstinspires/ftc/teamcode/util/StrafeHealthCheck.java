package org.firstinspires.ftc.teamcode.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TeleopTuningConfig;
import org.firstinspires.ftc.teamcode.util.HeadingHoldKpStore;

@Config
@TeleOp(name = "Strafe Health Check", group = "debug")
public class StrafeHealthCheck extends LinearOpMode {

    // ---- Tunables (Dashboard → Config) ----
    public static double STRAFE_POWER = 0.5;        // 0..1 strafe magnitude
    public static double SPIN_POWER   = 0.7;        // 0..1 for spin-rate calibration
    public static double SETTLE_S     = 0.30;       // ignore initial transient
    public static double TEST_S       = 2.00;       // measurement duration
    public static double TARGET_ERR_DEG = 5.0;      // desired steady-state error for Kp sizing

    // Application options
    public static boolean AUTO_APPLY_TO_CONFIG   = false; // write into TeleopTuningConfig now
    public static boolean AUTO_PERSIST_TO_DISK   = false; // write to file so TeleOp can load later
    public static boolean ROUND_KP               = true;
    public static int     KP_DECIMALS            = 2;

    private FtcDashboard dash;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        dash = FtcDashboard.getInstance();

        telemetry.addLine("This will run baseline strafe L/R, calibrate spin, suggest KP,");
        telemetry.addLine("then re-run with heading-hold using the suggested/applied KP.");
        telemetry.addLine("Optional: Press B after baseline to APPLY + PERSIST.");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // ---------- PASS 1: Baseline (rx = 0, no heading hold) ----------
        double driftLeftDegPerS  = measureYawRate(drive, +Math.abs(STRAFE_POWER), TEST_S);
        double driftRightDegPerS = measureYawRate(drive, -Math.abs(STRAFE_POWER), TEST_S);

        // Spin-rate calibration (deg/s at given rx)
        double driftSign = Math.abs(driftLeftDegPerS) >= Math.abs(driftRightDegPerS) ?
                Math.signum(driftLeftDegPerS) : Math.signum(driftRightDegPerS);
        double spinDegPerS = measureSpinRate(drive, driftSign * Math.abs(SPIN_POWER), TEST_S);
        double perUnitDegPerS = spinDegPerS / Math.max(1e-6, Math.abs(SPIN_POWER)); // deg/s for rx=+1

        double avgDriftDegPerS = (Math.abs(driftLeftDegPerS) + Math.abs(driftRightDegPerS)) / 2.0;
        double rxNeeded = (perUnitDegPerS > 1e-6) ? (avgDriftDegPerS / perUnitDegPerS) : 0.0;

        double targetErrRad = Math.toRadians(Math.max(1e-3, TARGET_ERR_DEG));
        double suggestedKp = rxNeeded / targetErrRad;
        if (ROUND_KP) suggestedKp = round(suggestedKp, KP_DECIMALS);

        // Option to apply/persist (in-memory)
        boolean applied = false, persisted = false;
        String codeLine = "TeleopTuningConfig.HEADING_HOLD_KP = " + format(suggestedKp, KP_DECIMALS) + ";";

        // Let driver decide: press B to apply + persist (or flip AUTO_* flags)
        long waitUntil = System.currentTimeMillis() + 4000;
        while (opModeIsActive() && System.currentTimeMillis() < waitUntil) {
            if (!applied && (AUTO_APPLY_TO_CONFIG || gamepad1.b)) {
                TeleopTuningConfig.HEADING_HOLD_KP = suggestedKp;
                applied = true;
                if (AUTO_PERSIST_TO_DISK || gamepad1.b) {
                    HeadingHoldKpStore.save(suggestedKp);
                    persisted = true;
                }
            }
            TelemetryPacket pkt = new TelemetryPacket();
            pkt.put("SUGGESTED_KP", suggestedKp);
            pkt.put("Press B to Apply+Persist", !AUTO_APPLY_TO_CONFIG);
            pkt.put("AppliedNow", applied);
            pkt.put("PersistedToDisk", persisted);
            pkt.put("Copy-Paste", codeLine);
            dash.sendTelemetryPacket(pkt);

            telemetry.addData("SUGGESTED HEADING_HOLD_KP", "%.2f", suggestedKp);
            telemetry.addData("AppliedNow", applied);
            telemetry.addData("PersistedToDisk", persisted);
            telemetry.addData("Copy-paste into code", codeLine);
            telemetry.update();
            idle();
        }

        // ---------- PASS 2: Re-test with heading hold (using applied or suggested Kp) ----------
        double kpForTest = applied ? TeleopTuningConfig.HEADING_HOLD_KP : suggestedKp;
        HoldMetrics leftHold  = measureStrafeWithHold(drive, +Math.abs(STRAFE_POWER), TEST_S, kpForTest);
        HoldMetrics rightHold = measureStrafeWithHold(drive, -Math.abs(STRAFE_POWER), TEST_S, kpForTest);

        // Stop
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        // ---------- Output ----------
        TelemetryPacket out = new TelemetryPacket();
        out.put("Baseline Drift L (deg/s)", driftLeftDegPerS);
        out.put("Baseline Drift R (deg/s)", driftRightDegPerS);
        out.put("Avg Baseline Drift (deg/s)", avgDriftDegPerS);
        out.put("Spin Rate (deg/s @power)", spinDegPerS);
        out.put("Per-Unit Spin (deg/s per rx=1)", perUnitDegPerS);
        out.put("SUGGESTED_KP", suggestedKp);
        out.put("AppliedNow", applied);
        out.put("PersistedToDisk", persisted);
        out.put("Copy-Paste", codeLine);

        out.put("ReTest KP Used", kpForTest);
        out.put("Hold LEFT  mean|err|(deg)", leftHold.meanAbsErrDeg);
        out.put("Hold LEFT  residual drift (deg/s)", leftHold.residualDriftDegPerS);
        out.put("Hold RIGHT mean|err|(deg)", rightHold.meanAbsErrDeg);
        out.put("Hold RIGHT residual drift (deg/s)", rightHold.residualDriftDegPerS);
        dash.sendTelemetryPacket(out);

        telemetry.addData("Baseline Drift L/R (deg/s)", "%.2f / %.2f", driftLeftDegPerS, driftRightDegPerS);
        telemetry.addData("Avg Baseline Drift (deg/s)", "%.2f", avgDriftDegPerS);
        telemetry.addData("Per-Unit Spin (deg/s/rx)", "%.2f", perUnitDegPerS);
        telemetry.addData("SUGGESTED_KP", "%.2f", suggestedKp);
        telemetry.addData("Applied / Persisted", "%s / %s", applied, persisted);
        telemetry.addData("ReTest KP Used", "%.2f", kpForTest);
        telemetry.addData("Hold L mean|err| (deg)", "%.2f", leftHold.meanAbsErrDeg);
        telemetry.addData("Hold L resid drift (deg/s)", "%.2f", leftHold.residualDriftDegPerS);
        telemetry.addData("Hold R mean|err| (deg)", "%.2f", rightHold.meanAbsErrDeg);
        telemetry.addData("Hold R resid drift (deg/s)", "%.2f", rightHold.residualDriftDegPerS);
        telemetry.update();

        sleep(2500);
    }

    // ---------- measurement helpers ----------

    /** Baseline: strafe at given power with rx=0, return avg yaw rate (deg/s). */
    private double measureYawRate(MecanumDrive drive, double strafePower, double durationS) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, strafePower), 0));
        sleep((long)(SETTLE_S * 1000));

        ElapsedTime t = new ElapsedTime();
        t.reset();
        double h0 = getHeadingRad(drive);
        while (opModeIsActive() && t.seconds() < durationS) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, strafePower), 0));
            drive.updatePoseEstimate();
            idle();
        }
        double h1 = getHeadingRad(drive);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        sleep(80);

        double dhDeg = Math.toDegrees(angleWrap(h1 - h0));
        return dhDeg / Math.max(1e-6, t.seconds());
    }

    /** Calibrate spin rate: command rx power for duration and return deg/s. */
    private double measureSpinRate(MecanumDrive drive, double rxPower, double durationS) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), rxPower));
        sleep((long)(SETTLE_S * 1000));

        ElapsedTime t = new ElapsedTime();
        t.reset();
        double h0 = getHeadingRad(drive);
        while (opModeIsActive() && t.seconds() < durationS) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), rxPower));
            drive.updatePoseEstimate();
            idle();
        }
        double h1 = getHeadingRad(drive);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        sleep(80);

        double dhDeg = Math.toDegrees(angleWrap(h1 - h0));
        return dhDeg / Math.max(1e-6, t.seconds());
    }

    /** Re-test: strafe with heading-hold controller (P only), return mean |err| and residual drift. */
    private HoldMetrics measureStrafeWithHold(MecanumDrive drive, double strafePower, double durationS, double kp) {
        // settle and set target heading to current
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, strafePower), 0));
        sleep((long)(SETTLE_S * 1000));
        double target = getHeadingRad(drive);

        ElapsedTime t = new ElapsedTime();
        t.reset();

        double sumAbsErr = 0;
        int samples = 0;
        double h0 = getHeadingRad(drive);

        while (opModeIsActive() && t.seconds() < durationS) {
            double h = getHeadingRad(drive);
            double err = angleWrap(target - h); // rad
            double rx = clamp(kp * err, -1.0, 1.0);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, strafePower), rx));

            drive.updatePoseEstimate();
            sumAbsErr += Math.abs(Math.toDegrees(err));
            samples++;
            idle();
        }

        double h1 = getHeadingRad(drive);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        sleep(80);

        HoldMetrics m = new HoldMetrics();
        m.meanAbsErrDeg = (samples > 0) ? (sumAbsErr / samples) : 0.0;
        m.residualDriftDegPerS = Math.toDegrees(angleWrap(h1 - h0)) / Math.max(1e-6, t.seconds());
        return m;
    }

    private static class HoldMetrics {
        double meanAbsErrDeg;
        double residualDriftDegPerS;
    }

    // ---------- math utils ----------
    private static double getHeadingRad(MecanumDrive d) {
        return d.localizer.getPose().heading.toDouble();
    }
    private static double angleWrap(double a) {
        while (a > Math.PI)  a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private static double round(double v, int decimals) {
        double s = Math.pow(10, decimals);
        return Math.round(v * s) / s;
    }
    private static String format(double v, int decimals) {
        return String.format("%." + decimals + "f", v);
    }
}
