package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Autonomous(name = "Auto: Obelisk → Spike → Launch → Face Goal", group = "auton")
public class AutoObeliskToLaunchRR extends LinearOpMode {

    // Pipelines per your setup
    private static final int PIPELINE_ALLTAGS   = 0;
    private static final int PIPELINE_OBELISK   = 1;
    private static final int PIPELINE_GOAL_BLUE = 2;
    private static final int PIPELINE_GOAL_RED  = 3;

    private static final String LIMELIGHT_NAME = "limelight";
    private static final long   MAX_STALENESS_MS = 500;
    private static final double YAW_ALIGN_DEG = 2.0;

    // Fallback (open-loop) controller if FF not tuned
    private static final boolean FF_TUNED = !(MecanumDrive.PARAMS.kS == 0 && MecanumDrive.PARAMS.kV == 0 && MecanumDrive.PARAMS.kA == 0);
    private static final double K_POS = 0.04, K_YAW = 0.02, LIN_CLAMP = 0.60, YAW_CLAMP = 0.40, STOP_POS_IN = 1.5, STOP_YAW_RAD = Math.toRadians(2.0);

    private enum Alliance { RED, BLUE }
    private enum Motif { A_I, A_II, A_III, UNKNOWN }

    private Alliance alliance = Alliance.RED; // default; toggle in INIT
    private Motif motif = Motif.UNKNOWN;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Limelight3A ll = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        try { ll.setPollRateHz(100); } catch (Throwable ignored) {}
        ll.pipelineSwitch(PIPELINE_OBELISK);
        ll.start();

        // ---- INIT: read motif + let driver pick alliance ----
        boolean lastLeft = false, lastRight = false;
        long t0 = System.currentTimeMillis();

        while (opModeInInit()) {
            // Alliance toggle
            boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
            if (left && !lastLeft)  alliance = Alliance.RED;
            if (right && !lastRight) alliance = Alliance.BLUE;
            lastLeft = left; lastRight = right;

            // Try to read motif from Obelisk pipeline
            LLResult res = ll.getLatestResult();
            String motifStr = "";
            if (res != null && res.isValid() && res.getStaleness() <= MAX_STALENESS_MS) {
                // Classifier (preferred)
                List<LLResultTypes.ClassifierResult> cls = res.getClassifierResults();
                if (cls != null && !cls.isEmpty()) {
                    // take the highest confidence classification
                    LLResultTypes.ClassifierResult best = cls.get(0);
                    for (LLResultTypes.ClassifierResult c : cls) {
                        if (c.getConfidence() > best.getConfidence()) best = c;
                    }
                    motifStr = best.getClassName();
                    motif = parseMotif(motifStr);
                } else {
                    // Detector fallback (if your obelisk pipeline is a detector)
                    List<LLResultTypes.DetectorResult> dets = res.getDetectorResults();
                    if (dets != null && !dets.isEmpty()) {
                        LLResultTypes.DetectorResult best = dets.get(0);
                        for (LLResultTypes.DetectorResult d : dets) {
                            if (d.getConfidence() > best.getConfidence()) best = d;
                        }
                        motifStr = best.getClassName();
                        motif = parseMotif(motifStr);
                    }
                }
            }

            telemetry.addData("Alliance", alliance);
            telemetry.addData("Motif", motif + (motifStr.isEmpty() ? "" : (" (" + motifStr + ")")));
            telemetry.addData("Pipeline", "ObeliskOnly(1) for motif. DPad LEFT=RED, RIGHT=BLUE.");
            telemetry.update();

            // (Optional) after 2s if nothing seen, briefly try AllTags to verify LL is streaming
            if (System.currentTimeMillis() - t0 > 2000 && motif == Motif.UNKNOWN) {
                ll.pipelineSwitch(PIPELINE_ALLTAGS);
                sleep(150);
                ll.pipelineSwitch(PIPELINE_OBELISK);
                t0 = System.currentTimeMillis();
            }
            sleep(20);
        }

        waitForStart();
        if (isStopRequested()) return;

        // Help MT2 with current yaw
        try {
            double yawDeg = Math.toDegrees(drive.localizer.getPose().heading.toDouble());
            ll.updateRobotOrientation(yawDeg);
        } catch (Throwable ignored) {}

        // ---- Segment 1: Drive from start to the correct SPIKE MARK ----
        Pose2d from = drive.localizer.getPose();
        Pose2d spike = spikePoseFor(alliance, motif);
        driveToPose(drive, from, spike);

        // (Insert any artifact placement / scoring you want here)

        // ---- Segment 2: Drive from SPIKE to FAR LAUNCH ZONE (e.g., D1 for RED) ----
        Pose2d launch = farLaunchPoseFor(alliance);
        driveToPose(drive, spike, launch);

        // ---- Segment 3: Face toward the GOAL using LL yaw (pipeline 2 blue / 3 red) ----
        int goalPipeline = (alliance == Alliance.RED) ? PIPELINE_GOAL_RED : PIPELINE_GOAL_BLUE;
        ll.pipelineSwitch(goalPipeline);
        sleep(150); // give it a moment

        yawAlignToGoal(drive, ll); // small open-loop yaw touch-up using tx

        // stop
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        drive.updatePoseEstimate();
    }

    private Motif parseMotif(String s) {
        if (s == null) return Motif.UNKNOWN;
        String t = s.trim().toLowerCase();
        if (t.contains("a.iii") || t.contains("aiii") || t.contains("a3")) return Motif.A_III;
        if (t.contains("a.ii")  || t.contains("aii")  || t.contains("a2")) return Motif.A_II;
        if (t.contains("a.i")   || t.contains("ai")   || t.contains("a1")) return Motif.A_I;
        return Motif.UNKNOWN;
    }

    // === YOU FILL THESE WITH REAL FIELD COORDINATES (inches, RR field frame) ===
    private Pose2d spikePoseFor(Alliance a, Motif m) {
        // TODO: replace these placeholders with your real spike positions for each motif/alliance
        // Example placeholders: x,y in inches; heading in radians
        if (a == Alliance.RED) {
            switch (m) {
                case A_I:   return new Pose2d(  41,  11, Math.toRadians(  0));
                case A_II:  return new Pose2d(  60,  40, Math.toRadians(  0));
                case A_III: return new Pose2d(  60,  50, Math.toRadians(  0));
                default:    return new Pose2d(  60,  40, Math.toRadians(  0)); // default center
            }
        } else { // BLUE
            switch (m) {
                case A_I:   return new Pose2d( -60,  30, Math.toRadians(180));
                case A_II:  return new Pose2d( -60,  40, Math.toRadians(180));
                case A_III: return new Pose2d( -60,  50, Math.toRadians(180));
                default:    return new Pose2d( -60,  40, Math.toRadians(180));
            }
        }
    }

    private Pose2d farLaunchPoseFor(Alliance a) {
        // TODO: replace with your actual far launch zone square pose (e.g., RED D1)
        if (a == Alliance.RED) {
            return new Pose2d(  96, -12, Math.toRadians(0));   // placeholder
        } else {
            return new Pose2d( -96, -12, Math.toRadians(180)); // placeholder
        }
    }
    // === end of coords you must fill ===

    private void driveToPose(MecanumDrive drive, Pose2d from, Pose2d goal) {
        if (FF_TUNED) {
            TrajectoryActionBuilder tab = drive.actionBuilder(from)
                    .splineToLinearHeading(goal, goal.heading.toDouble());
            Actions.runBlocking(tab.build());
        } else {
            // open-loop fallback (no FF tuning required)
            long tStart = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - tStart < 8000) {
                Pose2d pose = drive.localizer.getPose();
                double dx = goal.position.x - pose.position.x;
                double dy = goal.position.y - pose.position.y;

                double c = Math.cos(pose.heading.toDouble());
                double s = Math.sin(pose.heading.toDouble());
                double errFwd  =  c * dx + s * dy;
                double errLeft = -s * dx + c * dy;
                double errYaw  = wrap(goal.heading.toDouble() - pose.heading.toDouble());

                double fwd  = clamp(K_POS * errFwd,  -LIN_CLAMP, LIN_CLAMP);
                double left = clamp(K_POS * errLeft, -LIN_CLAMP, LIN_CLAMP);
                double yaw  = clamp(K_YAW * errYaw,  -YAW_CLAMP, YAW_CLAMP);

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, left), yaw));
                drive.updatePoseEstimate();

                if (Math.abs(errFwd) < STOP_POS_IN && Math.abs(errLeft) < STOP_POS_IN && Math.abs(errYaw) < STOP_YAW_RAD) {
                    break;
                }
                sleep(10);
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
            drive.updatePoseEstimate();
        }
    }

    private void yawAlignToGoal(MecanumDrive drive, Limelight3A ll) {
        // small open-loop yaw touch-up using LL tx on the goal pipeline
        long tStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - tStart < 2000) {
            LLResult res = ll.getLatestResult();
            if (res == null || !res.isValid() || res.getStaleness() > MAX_STALENESS_MS) break;

            List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
            if (fids == null || fids.isEmpty()) break;

            // Use the first fiducial on that goal (your pipeline isolates it)
            double txDeg = fids.get(0).getTargetXDegrees();
            double yawErr = Math.toRadians(txDeg);
            if (Math.abs(yawErr) < Math.toRadians(YAW_ALIGN_DEG)) break;

            double yawCmd = clamp(K_YAW * yawErr, -YAW_CLAMP, YAW_CLAMP);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), yawCmd));
            drive.updatePoseEstimate();
            sleep(10);
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        drive.updatePoseEstimate();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private static double wrap(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
}
