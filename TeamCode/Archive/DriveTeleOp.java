package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive TeleOp (Robot-Centric + Config + Overlay)", group = "drive")
public class DriveTeleOp extends LinearOpMode {

    private boolean fieldCentric = false; // off by default (you can toggle Y later)
    private boolean yPrev = false;
    private boolean aPrev = false;

    private double targetHeadingRad = 0.0;

    private DriveTeleopTelemetry overlay;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        overlay = new DriveTeleopTelemetry(200);
        overlay.setArrowScale(TeleopTuningConfig.ARROW_SCALE_IN);

        telemetry.addLine("Ready. (Y) Field/Robot, (A) reset yaw, (X) clear trail, RT slow mode, D-Pad snaps");
        telemetry.update();
        waitForStart();

        targetHeadingRad = drive.localizer.getPose().heading.toDouble();

        while (opModeIsActive() && !isStopRequested()) {

            double lx = gamepad1.left_stick_x;     // +right
            double ly = -gamepad1.left_stick_y;    // +forward
            double rx = -gamepad1.right_stick_x;    // +CCW

            // Deadzone + shaping
            lx = shape(deadzone(lx, TeleopTuningConfig.DEADZONE), TeleopTuningConfig.INPUT_CURVE_EXP);
            ly = shape(deadzone(ly, TeleopTuningConfig.DEADZONE), TeleopTuningConfig.INPUT_CURVE_EXP);
            rx = shape(deadzone(rx, TeleopTuningConfig.DEADZONE), TeleopTuningConfig.INPUT_CURVE_EXP);

            // Slow mode
            double slow = 1.0 - TeleopTuningConfig.SLOW_MAX_REDUCTION * gamepad1.right_trigger;
            lx *= slow; ly *= slow; rx *= slow;

            // Toggle field-centric on Y
            boolean yNow = gamepad1.y;
            if (yNow && !yPrev) fieldCentric = !fieldCentric;
            yPrev = yNow;

            Vector2d driveVec = new Vector2d(ly, -lx); // (forward, left)

            if (fieldCentric) {
                double heading = drive.localizer.getPose().heading.toDouble();
                double cos = Math.cos(-heading), sin = Math.sin(-heading);
                double f = driveVec.x * cos - driveVec.y * sin;
                double l = driveVec.x * sin + driveVec.y * cos;
                driveVec = new Vector2d(f, l);
            }

            // D-pad snaps (field frame)
            if (gamepad1.dpad_up)    targetHeadingRad = 0.0;
            if (gamepad1.dpad_right) targetHeadingRad = -Math.PI/2.0;
            if (gamepad1.dpad_down)  targetHeadingRad = Math.PI;
            if (gamepad1.dpad_left)  targetHeadingRad = Math.PI/2.0;

            double currentHeading = drive.localizer.getPose().heading.toDouble();

            // Heading hold when driver isn't turning
            if (Math.abs(rx) < TeleopTuningConfig.HEADING_HOLD_DEAD) {
                double err = wrap(targetHeadingRad - currentHeading);
                double u = TeleopTuningConfig.HEADING_HOLD_KP * err;
                rx = clamp(u, -1.0, 1.0) * slow;
            } else {
                targetHeadingRad = currentHeading;
            }

            // Drive
            drive.setDrivePowers(new PoseVelocity2d(driveVec, rx));
            drive.updatePoseEstimate();

            // Yaw reset
            boolean aNow = gamepad1.a;
            if (aNow && !aPrev) {
                try { drive.lazyImu.get().resetYaw(); } catch (Exception ignored) {}
                targetHeadingRad = 0.0;
            }
            aPrev = aNow;

            // Clear trail
            if (gamepad1.x) overlay.resetTrail();

            // Overlay
            overlay.send(drive, new Vector2d(driveVec.x, driveVec.y), rx, fieldCentric, slow);

            // DS telemetry
            Pose2d p = drive.localizer.getPose();
            telemetry.addData("Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
            telemetry.addData("Pose (in)", "x=%.2f  y=%.2f  θ=%.1f°",
                    p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
            telemetry.addData("Target θ", "%.1f°", Math.toDegrees(targetHeadingRad));
            telemetry.addData("Cmd", "f=%.2f  l=%.2f  ω=%.2f", driveVec.x, driveVec.y, rx);
            telemetry.addData("Slow", "%.0f%%", slow * 100.0);
            telemetry.update();
        }
    }

    // helpers
    private static double deadzone(double v, double dz) {
        return Math.abs(v) < dz ? 0.0 : v;
    }
    private static double shape(double v, double exp) {
        // generalized curve; exp=2 is square, 3 is cubic-ish
        double a = Math.abs(v);
        double out = Math.pow(a, exp);
        return Math.copySign(out, v);
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private static double wrap(double a) {
        while (a > Math.PI)  a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
}
