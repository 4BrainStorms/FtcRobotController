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

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.StartPoses;

import java.util.List;

/** Drive to a pose 24" in front of Tag 24 (facing it). Uses RR if FF tuned, else open-loop fallback. */
@Autonomous(name = "Tag24 Field Approach (RR or fallback)", group = "auton")
public class Tag24FieldApproachRR extends LinearOpMode {

    public static final String LIMELIGHT_NAME = "limelight";
    public static final int PIPELINE_ALLTAGS = 0;
    public static final int PIPELINE_TAG24   = 3;
    public static final double STANDOFF_IN   = 24.0;
    public static final long   MAX_STALENESS_MS = 500;

    // Fallback (open-loop) when FF not tuned
    private static final double K_POS = 0.04, K_YAW = 0.02;
    private static final double LIN_CLAMP = 0.60, YAW_CLAMP = 0.40;
    private static final double STOP_POS_IN = 1.5, STOP_YAW_RAD = Math.toRadians(2.0);
    private static final long   FALLBACK_TIMEOUT_MS = 8000;

    @Override
    public void runOpMode() {
        // <-- start pose from your “front on 1, right on D–E” definition
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPoses.START_D1);

        Limelight3A ll = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        try { ll.setPollRateHz(100); } catch (Throwable ignored) {}
        ll.pipelineSwitch(PIPELINE_TAG24);
        ll.start();

        // INIT: show LL status; fallback to AllTags after 2s if needed
        long initStart = System.currentTimeMillis();
        boolean saw24 = false; int pipelineNow = PIPELINE_TAG24;
        while (opModeInInit()) {
            LLResult res = ll.getLatestResult();
            int count = -1; long stale = -1;
            if (res != null) {
                stale = res.getStaleness();
                if (res.isValid()) {
                    List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
                    if (fids != null) {
                        count = fids.size();
                        for (LLResultTypes.FiducialResult f : fids) if (f.getFiducialId()==24) saw24 = true;
                    }
                }
            }
            telemetry.addData("LL valid", res != null && res.isValid());
            telemetry.addData("staleness(ms)", stale);
            telemetry.addData("fiducials", count);
            telemetry.addData("pipeline", pipelineNow);
            telemetry.addData("Tag24", saw24 ? "VISIBLE" : "not yet");
            telemetry.update();

            if (!saw24 && System.currentTimeMillis()-initStart>2000 && pipelineNow!=PIPELINE_ALLTAGS) {
                ll.pipelineSwitch(PIPELINE_ALLTAGS);
                pipelineNow = PIPELINE_ALLTAGS;
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

        final double M_TO_IN = 39.37007874015748;
        Pose2d start = drive.localizer.getPose();

        // Acquire Tag 24 (≤5s)
        LLResultTypes.FiducialResult f24 = null; long t0 = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis()-t0)<5000) {
            LLResult res = ll.getLatestResult();
            if (res != null && res.isValid() && res.getStaleness() <= MAX_STALENESS_MS) {
                List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
                if (fids != null) for (LLResultTypes.FiducialResult f : fids) {
                    if (f.getFiducialId()==24) { f24=f; break; }
                }
            }
            if (f24 != null) break;
        }
        if (f24 == null) { telemetry.addLine("Tag 24 not found"); telemetry.update(); return; }

        // Robot pose in TAG frame (meters): X right, Z out
        Pose3D rps = f24.getRobotPoseTargetSpace();
        double x_right_in = rps.getPosition().x * M_TO_IN;
        double z_out_in   = rps.getPosition().z * M_TO_IN;

        // Use same sign convention as your working servo:
        // forward offset ∝ -(z - standoff), strafe offset ∝ -x
        double move_fx = -(z_out_in - STANDOFF_IN);
        double move_fy = -x_right_in;

        // ROBOT→WORLD using current heading
        Pose2d rr = drive.localizer.getPose();
        double ch = Math.cos(rr.heading.toDouble()), sh = Math.sin(rr.heading.toDouble());
        double gx = rr.position.x + (ch * move_fx - sh * move_fy);
        double gy = rr.position.y + (sh * move_fx + ch * move_fy);

        // Face the tag using tx
        double yawErrRad = Math.toRadians(f24.getTargetXDegrees());
        double gHeading  = rr.heading.toDouble() + yawErrRad;

        boolean ffTuned = !(MecanumDrive.PARAMS.kS == 0 && MecanumDrive.PARAMS.kV == 0 && MecanumDrive.PARAMS.kA == 0);

        if (ffTuned) {
            TrajectoryActionBuilder tab = drive.actionBuilder(start)
                    .splineToLinearHeading(new Pose2d(gx, gy, gHeading), gHeading);
            Actions.runBlocking(tab.build());

            // tiny yaw touch-up
            LLResult res = ll.getLatestResult();
            if (res != null && res.isValid() && res.getStaleness() <= MAX_STALENESS_MS) {
                List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
                if (fids != null) for (LLResultTypes.FiducialResult f : fids) {
                    if (f.getFiducialId()==24) {
                        double yaw2 = Math.toRadians(f.getTargetXDegrees());
                        if (Math.abs(yaw2) > Math.toRadians(2)) {
                            Action finalTurn = drive.actionBuilder(drive.localizer.getPose())
                                    .turn(yaw2).build();
                            Actions.runBlocking(finalTurn);
                        }
                        break;
                    }
                }
            }
        } else {
            // Open-loop fallback to the goal pose
            long tStart = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis()-tStart < FALLBACK_TIMEOUT_MS) {
                Pose2d pose = drive.localizer.getPose();
                double dx = gx - pose.position.x, dy = gy - pose.position.y;
                double c = Math.cos(pose.heading.toDouble()), s = Math.sin(pose.heading.toDouble());
                double errFwd  =  c*dx + s*dy;
                double errLeft = -s*dx + c*dy;
                double errYaw  = wrap(gHeading - pose.heading.toDouble());

                double fwd  = clamp(K_POS*errFwd,  -LIN_CLAMP, LIN_CLAMP);
                double left = clamp(K_POS*errLeft, -LIN_CLAMP, LIN_CLAMP);
                double yaw  = clamp(K_YAW*errYaw,  -YAW_CLAMP, YAW_CLAMP);

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, left), yaw));
                drive.updatePoseEstimate();

                if (Math.abs(errFwd) < STOP_POS_IN &&
                        Math.abs(errLeft) < STOP_POS_IN &&
                        Math.abs(errYaw) < STOP_YAW_RAD) break;

                sleep(10);
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
            drive.updatePoseEstimate();
        }
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double wrap(double a){ while(a>Math.PI)a-=2*Math.PI; while(a<-Math.PI)a+=2*Math.PI; return a; }
}
