package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d; // <-- If your package is com.acmerobotics.* keep the 'acmerobotics' spelling!
// NOTE: If your RR import is com.acmerobotics.roadrunner.Vector2d (normal), replace the line above with that.
// Some editors auto-correct; ensure your Vector2d import matches your project.

import com.acmerobotics.roadrunner.Vector2d; // typical RR import

// Limelight3A FTC USB API
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

// For fiducial camera-space pose (if exposed on your SDK build)
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.lang.reflect.Method;
import java.util.List;

@TeleOp(name = "Drive + Tag24 (Pinpoint 60in on Y)", group = "drive")
public class DriveTeleOp_Tag24_Pinpoint60in extends LinearOpMode {

    // ===== Driver shaping (yours) =====
    private static final double DEADZONE  = 0.05;
    private static final double INPUT_EXP = 2.0;
    private static final double SLOW_MIN  = 0.35;

    // ===== Geometry & pipeline =====
    private static final int    PIPELINE_INDEX         = 3;      // AprilTag pipeline (ID 24)
    private static final double CAMERA_TO_BUMPER_IN    = 5.0;    // camera is 5" behind bumper
    private static final double BUMPER_TARGET_IN       = 60.0;   // requested bumper distance
    private static final double TARGET_RANGE_IN        = BUMPER_TARGET_IN + CAMERA_TO_BUMPER_IN; // 65" cam->tag
    private static final double IN_PER_M               = 39.37007874;

    // ===== Coarse ALIGN (no P-loop twitch) =====
    private static final double ALIGN_ENTER_DEG        = 6.0;    // turn while |tx| > 6°
    private static final double ALIGN_EXIT_DEG         = 3.0;    // centered when |tx| <= 3°
    private static final long   ALIGN_STABLE_MS        = 2000;    // must hold centered for this long
    private static final double ALIGN_TURN_SPEED       = 0.10;   // constant turn speed in ALIGN

    // ===== Search (slow + dwell) =====
    private static final double SEARCH_TURN            = 0.04;
    private static final long   SEARCH_DWELL_MS        = 2000;

    // Vision lost timeout
    private static final long   LOST_TIMEOUT_MS        = 3000;

    // ===== Straight advance & final creep =====
    private static final double KP_FWD                 = 0.030;
    private static final double MAX_FWD                = 0.35;   // slower forward for stable frames
    private static final double FINAL_WINDOW_IN        = 12.0;
    private static final double FINAL_TURN_MAX         = 0.18;   // very gentle final turn
    private static final double FINAL_TURN_KP          = 0.010;  // very low gain near goal
    private static final double MAX_FWD_NEAR           = 0.25;   // creep
    private static final double RANGE_TOL_IN           = 1.0;

    // ===== Safety =====
    private static final double MIN_RANGE_IN           = 48.0;   // too-close guard

    // ===== Safe-Stop: Odometry & Image-Area backstops =====
    private static final double APPROACH_BUDGET_MAX_IN = 84.0;   // absolute max allowed travel in ADVANCE
    private static final long   APPROACH_TIMEOUT_MS    = 3000;   // stop if ADVANCE exceeds this time
    private static final double TA_STOP_PCT            = 8.0;    // stop if ta (%) >= this (tune once with telemetry)

    // ===== Range-unavailable helper (short pulses) =====
    private static final long   NAR_PULSE_FWD_MS       = 250;
    private static final long   NAR_PULSE_HOLD_MS      = 250;
    private static final double NAR_PULSE_SPEED        = 0.20;

    // ===== State =====
    private enum State { SEARCH, ALIGN, ADVANCE, FINAL_ALIGN, DONE }
    private State state = State.SEARCH;

    // ===== Timing: last valid LL frame (fix for your compile error) =====
    private long lastValidMs = 0L;

    // ===== Memorized =====
    private double lastTxDeg = 0.0;
    private long   alignStableStartMs = 0;
    private boolean prevVisible = false;

    // Search dwell
    private long   lastSearchDirChangeMs = 0;
    private double searchDirHold = -1.0;

    // ADVANCE budget & timing
    private double advanceStartX = 0.0, advanceStartY = 0.0;
    private long   advanceStartMs = 0L;
    private double approachBudgetIn = 0.0;
    private boolean narForwardPhase = true;
    private long   narPulseStartMs = 0L;

    // Heading lock for straight drive (we keep KP_YAW_HOLD = 0 for perfectly straight)
    private double headingLockDeg = 0.0;

    @Override
    public void runOpMode() {

        // --- Your drive as-is ---
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        DcMotor extraMotor = hardwareMap.get(DcMotor.class, "intake");

        // --- Limelight3A init ---
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(PIPELINE_INDEX);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Y: Coarse-align → Straight to 60\" bumper → Gentle finish (safe stop).");
        telemetry.update();

        waitForStart();

        boolean assistActive = false;
        lastValidMs = System.currentTimeMillis();

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.y) { assistActive = true;  state = State.SEARCH; alignStableStartMs = 0; }
            if (gamepad1.b) { assistActive = false; state = State.SEARCH; }

            LLResult result = limelight.getLatestResult();
            boolean visible = (result != null && result.isValid());
            if (visible) lastValidMs = System.currentTimeMillis();

            if (visible && !prevVisible) { try { gamepad1.rumble(200); } catch (Exception ignored) {} }
            prevVisible = visible;

            // ===== Manual drive path =====
            if (!assistActive) {
                double fwd  = -gamepad1.left_stick_y;
                double left = -gamepad1.left_stick_x;
                double turn = -gamepad1.right_stick_x;

                fwd  = shape(deadzone(fwd,  DEADZONE), INPUT_EXP);
                left = shape(deadzone(left, DEADZONE), INPUT_EXP);
                turn = shape(deadzone(turn, DEADZONE), INPUT_EXP);

                double slow = lerp(1.0, SLOW_MIN, clamp01(gamepad1.right_trigger));
                fwd  *= slow; left *= slow; turn *= slow;

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, left), turn));
                drive.updatePoseEstimate();

                if (gamepad1.left_trigger > 0.1) extraMotor.setPower(-1.0); else extraMotor.setPower(0.0);
                if (gamepad1.a) { try { drive.lazyImu.get().resetYaw(); } catch (Exception ignored) {} }

                Pose2d p = drive.localizer.getPose();
                telemetry.addData("Mode", "Manual");
                telemetry.addData("Pose (in)", "x=%.2f y=%.2f θ=%.1f°",
                        p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
                pushTagTelemetry(result);
                telemetry.addData("Target(camera|bumper)", "%.1f | %.1f in", TARGET_RANGE_IN, BUMPER_TARGET_IN);
                telemetry.update();
                continue;
            }

            // ===== Assisted path =====
            double tx = 0.0;
            double ta = 0.0;
            Double rangeIn = null;   // inches; null if unavailable

            if (visible) {
                tx = result.getTx();  // + => tag right of crosshair (Limelight FTC)
                ta = result.getTa();
                lastTxDeg = tx;
                rangeIn = estimateRangeInches(result); // botpose distance OR fiducial camera-space Z
            }

            // Too-close guard
            if (rangeIn != null && rangeIn < MIN_RANGE_IN) {
                safeStop(drive);
                telemetry.addData("State", "DONE (too close)");
                pushTagTelemetry(result);
                telemetry.update();
                continue;
            }

            double yawDeg = Math.toDegrees(drive.localizer.getPose().heading.toDouble());
            double fwdCmd = 0.0, turnCmd = 0.0;

            switch (state) {
                case SEARCH: {
                    if (visible) {
                        state = State.ALIGN;
                        alignStableStartMs = 0;
                    } else {
                        long now = System.currentTimeMillis();
                        double desiredDir = (lastTxDeg == 0.0) ? -1.0 : -Math.signum(lastTxDeg); // lastTx>0 => CW (neg)
                        if (now - lastSearchDirChangeMs > SEARCH_DWELL_MS) {
                            searchDirHold = desiredDir;
                            lastSearchDirChangeMs = now;
                        }
                        turnCmd = SEARCH_TURN * searchDirHold;
                        fwdCmd = 0.0;
                    }
                    break;
                }

                case ALIGN: {
                    if (!visible) {
                        if (System.currentTimeMillis() - lastValidMs > LOST_TIMEOUT_MS) state = State.SEARCH;
                        break;
                    }
                    double absTx = Math.abs(tx);
                    if (absTx > ALIGN_ENTER_DEG) {
                        // No P-loop: fixed, slow turn in correct direction
                        turnCmd = (tx > 0) ? -ALIGN_TURN_SPEED : +ALIGN_TURN_SPEED;
                        alignStableStartMs = 0;
                    } else if (absTx <= ALIGN_EXIT_DEG) {
                        if (alignStableStartMs == 0) alignStableStartMs = System.currentTimeMillis();
                        turnCmd = 0.0; // hold still to let solver stabilize
                        if (System.currentTimeMillis() - alignStableStartMs >= ALIGN_STABLE_MS) {
                            headingLockDeg = yawDeg;
                            // Initialize ADVANCE budget using odometry
                            Pose2d p0 = drive.localizer.getPose();
                            advanceStartX = p0.position.x;
                            advanceStartY = p0.position.y;
                            advanceStartMs = System.currentTimeMillis();
                            if (rangeIn != null) {
                                // distance to drive = rangeAtAlign - 65", plus small buffer for braking
                                approachBudgetIn = clip(rangeIn - TARGET_RANGE_IN + 4.0, 0.0, APPROACH_BUDGET_MAX_IN);
                            } else {
                                approachBudgetIn = 40.0; // conservative default if range N/A
                            }
                            narPulseStartMs = 0;
                            state = State.ADVANCE;
                        }
                    } else {
                        // Between ENTER & EXIT: hold still to avoid chatter
                        turnCmd = 0.0;
                        alignStableStartMs = 0;
                    }
                    fwdCmd = 0.0;
                    break;
                }

                case ADVANCE: {
                    // Measure forward progress along headingLock (projection);
                    Pose2d p = drive.localizer.getPose();
                    double headingLockRad = Math.toRadians(headingLockDeg);
                    double dx = p.position.x - advanceStartX;
                    double dy = p.position.y - advanceStartY;
                    double forwardIn =  dx * Math.cos(headingLockRad) + dy * Math.sin(headingLockRad);
                    long   elapsedMs  = System.currentTimeMillis() - advanceStartMs;

                    // Hard backstops (stop even if vision is flaky)
                    boolean budgetHit  = forwardIn >= approachBudgetIn;
                    boolean timeoutHit = elapsedMs  >= APPROACH_TIMEOUT_MS;
                    boolean areaHit    = ta >= TA_STOP_PCT;

                    if (budgetHit || timeoutHit || areaHit) {
                        if (visible) {
                            state = State.FINAL_ALIGN;
                            fwdCmd = 0.0; turnCmd = 0.0;
                            break;
                        } else {
                            safeStop(drive);
                            telemetry.addData("State", "DONE (budget/timeout/area)");
                            pushTagTelemetry(result);
                            telemetry.addData("Travel/Alloc (in)", "%.1f / %.1f", forwardIn, approachBudgetIn);
                            telemetry.update();
                            continue;
                        }
                    }

                    if (visible && rangeIn != null) {
                        double rangeErr = rangeIn - TARGET_RANGE_IN;
                        fwdCmd = (rangeErr > RANGE_TOL_IN) ? clip(KP_FWD * rangeErr, 0.0, MAX_FWD) : 0.0;
                        turnCmd = 0.0; // perfectly straight (no tx steering)
                        if (rangeIn <= TARGET_RANGE_IN + FINAL_WINDOW_IN) state = State.FINAL_ALIGN;
                    } else {
                        // Help pose lock: short forward pulse, then hold
                        long now = System.currentTimeMillis();
                        if (narPulseStartMs == 0) { narPulseStartMs = now; narForwardPhase = true; }
                        long dt = now - narPulseStartMs;
                        if (narForwardPhase) {
                            fwdCmd = NAR_PULSE_SPEED; turnCmd = 0.0;
                            if (dt >= NAR_PULSE_FWD_MS) { narPulseStartMs = now; narForwardPhase = false; }
                        } else {
                            fwdCmd = 0.0; turnCmd = 0.0;
                            if (dt >= NAR_PULSE_HOLD_MS) { narPulseStartMs = now; narForwardPhase = true; }
                        }
                    }

                    telemetry.addData("ADV Forward/Alloc (in)", "%.1f / %.1f", forwardIn, approachBudgetIn);
                    telemetry.addData("ADV Time (ms)", elapsedMs);
                    break;
                }

                case FINAL_ALIGN: {
                    if (!visible) {
                        if (System.currentTimeMillis() - lastValidMs > LOST_TIMEOUT_MS) { safeStop(drive); continue; }
                        break;
                    }
                    // Gentle in-place alignment near goal
                    double desiredYawFinal = yawDeg - tx;  // tx>0 => need CW (neg)
                    double errDeg = angleDiffDeg(desiredYawFinal, yawDeg);
                    turnCmd = clip(FINAL_TURN_KP * errDeg, -FINAL_TURN_MAX, FINAL_TURN_MAX);
                    fwdCmd = 0.0;

                    if (rangeIn != null) {
                        double rangeErr = rangeIn - TARGET_RANGE_IN;
                        boolean facingOK = Math.abs(tx) <= ALIGN_EXIT_DEG;
                        boolean rangeOK  = Math.abs(rangeErr) <= RANGE_TOL_IN;

                        if (facingOK) {
                            if (!rangeOK && rangeErr > 0) {
                                fwdCmd = clip(KP_FWD * rangeErr, 0.0, MAX_FWD_NEAR);
                                turnCmd = 0.0; // creep straight
                            } else if (rangeOK) {
                                state = State.DONE;
                            }
                        }
                    } else {
                        // If range still N/A, use ta or odometry budget (already consumed) as fallback
                        if (ta >= TA_STOP_PCT) state = State.DONE;
                    }
                    break;
                }

                case DONE: {
                    fwdCmd = 0.0; turnCmd = 0.0;
                    assistActive = false;
                    state = State.SEARCH;
                    break;
                }
            }

            // Command drivetrain (no strafe during assist)
            if (assistActive) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwdCmd, 0.0), turnCmd));
                drive.updatePoseEstimate();

                telemetry.addData("State", state);
                pushTagTelemetry(result);
                telemetry.addData("Cmd", "fwd=%.2f turn=%.2f", fwdCmd, turnCmd);
                telemetry.addData("Target(camera|bumper)", "%.1f | %.1f in", TARGET_RANGE_IN, BUMPER_TARGET_IN);
                telemetry.update();
            }

            // Keep intake + A reset always available
            if (gamepad1.left_trigger > 0.1) extraMotor.setPower(-1.0); else extraMotor.setPower(0.0);
            if (gamepad1.a) { try { drive.lazyImu.get().resetYaw(); } catch (Exception ignored) {} }
        }
    }

    // ---- Safe stop helper ----
    private void safeStop(MecanumDrive drive) {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        drive.updatePoseEstimate();
    }

    // ---- Range estimator: botpose (m) -> in; else fiducial camera-space Z (m) -> in; else null ----
    private Double estimateRangeInches(LLResult r) {
        if (r == null) return null;

        // 1) botpose average distance (meters) – per FTC Limelight Javadoc
        double m = r.getBotposeAvgDist();
        if (m > 0) return m * IN_PER_M;

        // 2) fallback: fiducial camera-space Z (Pose3D); method names vary by SDK build
        try {
            List<LLResultTypes.FiducialResult> fids = r.getFiducialResults();
            if (fids != null && !fids.isEmpty()) {
                LLResultTypes.FiducialResult fr = fids.get(0);

                // Preferred accessor (if present on your build)
                try {
                    Method getPose = fr.getClass().getMethod("getTargetPose_CameraSpace");
                    Object poseObj = getPose.invoke(fr);
                    if (poseObj instanceof Pose3D) {
                        Pose3D p = (Pose3D) poseObj;
                        double zMeters = p.getPosition().z;
                        if (zMeters > 0) return zMeters * IN_PER_M;
                    }
                } catch (NoSuchMethodException ignored) {}

                // Alternate names seen in some builds
                String[] zMethods = { "getDistToCamera", "getDistanceToCamera", "getZ", "getTargetZMeters" };
                for (String name : zMethods) {
                    try {
                        Method mz = fr.getClass().getMethod(name);
                        Object val = mz.invoke(fr);
                        if (val instanceof Double) {
                            double zMeters = (Double) val;
                            if (zMeters > 0) return zMeters * IN_PER_M;
                        }
                    } catch (NoSuchMethodException ignored) {}
                }
            }
        } catch (Exception ignored) {}

        return null; // unavailable
    }

    // ---- Telemetry HUD (always shows Range) ----
    private void pushTagTelemetry(LLResult r) {
        boolean valid = (r != null && r.isValid());
        telemetry.addData("VISIBLE", valid ? "YES \u2714" : "no");
        telemetry.addData("Pipeline", PIPELINE_INDEX);

        if (r != null) {
            telemetry.addData("tx/ty (deg)", "%.1f / %.1f", r.getTx(), r.getTy());
            telemetry.addData("ta (%%)", "%.2f", r.getTa());

            Double rng = estimateRangeInches(r);
            if (rng != null) telemetry.addData("Range est (in)", "%.1f", rng);
            else telemetry.addData("Range est (in)", "N/A (enable 3D or fiducial Z)");

            telemetry.addData("TagCount(botpose)", r.getBotposeTagCount());
            telemetry.addData("Latency (ms)", "cap=%.0f pipe=%.0f", r.getCaptureLatency(), r.getTargetingLatency());
        } else {
            telemetry.addData("tx/ty (deg)", "-- / --");
            telemetry.addData("ta (%)", "--");
            telemetry.addData("Range est (in)", "N/A (no latest result)");
        }
    }

    // ===== Helpers (from your base TeleOp) =====
    private static double deadzone(double v, double dz) { return Math.abs(v) < dz ? 0.0 : v; }
    private static double shape(double v, double exp) {
        double a = Math.abs(v);
        double out = Math.pow(a, exp);
        return Math.copySign(out, v);
    }
    private static double clamp01(double v){ return Math.max(0.0, Math.min(1.0, v)); }
    private static double lerp(double a, double b, double t){ return a + (b - a) * t; }
    private static double clip(double v, double min, double max){ return Math.max(min, Math.min(max, v)); }

    /** Signed, shortest angular difference target - current, in degrees, into [-180, +180]. */
    private static double angleDiffDeg(double targetDeg, double currentDeg) {
        double diff = targetDeg - currentDeg;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }
}
