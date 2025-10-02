package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.StartPoses;

import java.util.List;

@Autonomous(name = "Tag24 Servo Approach (Pinpoint-led)", group = "auton")
public class Tag24ServoApproach extends LinearOpMode {

    // ---- Limelight & approach settings ----
    public static final String LIMELIGHT_NAME = "limelight";
    public static final int    PIPELINE_TAG24 = 3;        // your Goal_Red_24 pipeline
    public static final long   MAX_STALENESS_MS = 200;    // reject stale frames
    public static final long   LL_GRACE_MS     = 400;     // keep chasing last goal for this long if LL drops

    public static final double STANDOFF_IN = 24.0;        // desired final distance from tag face

    // Robot-frame holonomic P (open-loop power since kS/kV/kA not tuned yet)
    public static final double K_POS = 0.04;              // pos -> power (in -> power)
    public static final double K_YAW = 0.02;              // rad -> power
    public static final double LIN_CLAMP = 0.60;          // max |fwd|, |left|
    public static final double YAW_CLAMP = 0.40;          // max |yaw|

    // Stop when we're close enough (using Pinpoint pose vs goal)
    public static final double STOP_POS_IN  = 1.5;
    public static final double STOP_YAW_RAD = Math.toRadians(2.0);

    @Override
    public void runOpMode() {
        // Start with your D1 placement (corner-origin frame)
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPoses.START_D1);

        Limelight3A ll = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        try { ll.setPollRateHz(100); } catch (Throwable ignored) {}
        ll.pipelineSwitch(PIPELINE_TAG24);
        ll.start();

        telemetry.addLine("Init → looking for Tag 24…");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        final double M_TO_IN = 39.37007874015748;

        // Last target pose we computed from LL (field frame)
        Pose2d lastGoal = drive.localizer.getPose();
        long lastGoalTs = 0;

        long tStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - tStart < 9000) {

            // Keep LL aware of our current yaw for pose solves
            try {
                double yawDeg = Math.toDegrees(drive.localizer.getPose().heading.toDouble());
                ll.updateRobotOrientation(yawDeg);
            } catch (Throwable ignored) {}

            // 1) Try to get a fresh Tag 24 observation
            boolean haveFreshGoal = false;
            LLResult res = ll.getLatestResult();
            if (res != null && res.isValid() && res.getStaleness() <= MAX_STALENESS_MS) {
                List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
                if (fids != null) {
                    for (LLResultTypes.FiducialResult f : fids) {
                        if (f.getFiducialId() == 24) {
                            // Robot pose in TAG frame (m): X right, Y down, Z out of tag
                            Pose3D rps = f.getRobotPoseTargetSpace();
                            double x_right_in = rps.getPosition().x * M_TO_IN;
                            double z_out_in   = rps.getPosition().z * M_TO_IN;

                            // Invert to get robot->tag in ROBOT frame (planar)
                            double yaw_rel = rps.getOrientation().getYaw(AngleUnit.RADIANS);
                            double cy = Math.cos(-yaw_rel), sy = Math.sin(-yaw_rel);
                            double fwd_rt  = -( cy * x_right_in - sy * z_out_in); // +X forward
                            double left_rt = -( sy * x_right_in + cy * z_out_in); // +Y left

                            // Desired delta in ROBOT frame so we end STANDOFF_IN from the tag, facing it
                            double d = Math.hypot(fwd_rt, left_rt);
                            double ufx = (d > 1e-6) ? fwd_rt / d : 1.0;
                            double ufy = (d > 1e-6) ? left_rt / d : 0.0;
                            double move_fx = fwd_rt - STANDOFF_IN * ufx;
                            double move_fy = left_rt - STANDOFF_IN * ufy;

                            // Convert that delta to WORLD using current Pinpoint heading
                            Pose2d now = drive.localizer.getPose();
                            double ch = Math.cos(now.heading.toDouble());
                            double sh = Math.sin(now.heading.toDouble());
                            double dx_world = ch * move_fx - sh * move_fy;
                            double dy_world = sh * move_fx + ch * move_fy;

                            // Global goal pose (position) + desired heading to face the tag
                            double gx = now.position.x + dx_world;
                            double gy = now.position.y + dy_world;
                            double heading_to_tag_rel_robot = Math.atan2(left_rt, fwd_rt);
                            double gHeading = now.heading.toDouble() + heading_to_tag_rel_robot;

                            lastGoal   = new Pose2d(gx, gy, gHeading);
                            lastGoalTs = System.currentTimeMillis();
                            haveFreshGoal = true;

                            telemetry.addData("LL ok", "dist=%.1f in", d);
                            break;
                        }
                    }
                }
            }

            // 2) If no fresh tag for a short window, keep driving toward last goal using Pinpoint
            boolean goalValid = haveFreshGoal || (System.currentTimeMillis() - lastGoalTs) <= LL_GRACE_MS;

            // 3) Pinpoint-led P control to the goal (robot-frame error → powers)
            Pose2d pose = drive.localizer.getPose();
            double dx = lastGoal.position.x - pose.position.x;
            double dy = lastGoal.position.y - pose.position.y;

            double c = Math.cos(pose.heading.toDouble());
            double s = Math.sin(pose.heading.toDouble());
            double errFwd  =  c*dx + s*dy;
            double errLeft = -s*dx + c*dy;

            double errYaw  = wrap(lastGoal.heading.toDouble() - pose.heading.toDouble());

            double fwd  = clamp(K_POS * errFwd,  -LIN_CLAMP, LIN_CLAMP);
            double left = clamp(K_POS * errLeft, -LIN_CLAMP, LIN_CLAMP);
            double yaw  = clamp(K_YAW * errYaw,  -YAW_CLAMP, YAW_CLAMP);

            if (!goalValid) { fwd = 0; left = 0; yaw = 0; }

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, left), yaw));
            drive.updatePoseEstimate();

            telemetry.addData("goal", "x=%.1f y=%.1f θ=%.1f°",
                    lastGoal.position.x, lastGoal.position.y, Math.toDegrees(lastGoal.heading.toDouble()));
            telemetry.addData("pose", "x=%.1f y=%.1f θ=%.1f°",
                    pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("cmd",  "f=%.2f l=%.2f w=%.2f", fwd, left, yaw);
            telemetry.addData("LL fresh", haveFreshGoal);
            telemetry.update();

            if (goalValid &&
                    Math.hypot(errFwd, errLeft) < STOP_POS_IN &&
                    Math.abs(errYaw) < STOP_YAW_RAD) {
                break;
            }

            sleep(10);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        drive.updatePoseEstimate();
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double wrap(double a){ while(a>Math.PI)a-=2*Math.PI; while(a<-Math.PI)a+=2*Math.PI; return a; }
}
