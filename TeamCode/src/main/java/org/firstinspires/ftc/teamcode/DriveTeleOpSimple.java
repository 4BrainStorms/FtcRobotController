package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive TeleOp (Robot-Centric — Simple)", group = "drive")
public class DriveTeleOpSimple extends LinearOpMode {

    // tweak if you like
    private static final double DEADZONE = 0.05;
    private static final double INPUT_EXP = 2.0;       // 2 = square, softer around center
    private static final double SLOW_MIN = 0.35;       // RT fully pressed scales to 35%

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Robot-centric drive. RT = slow mode, A = reset yaw.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // raw sticks (FTC: up is negative on Y)
            double fwd  = -gamepad1.left_stick_y;   // +forward
            double left = -gamepad1.left_stick_x;   // +left strafe
            double turn = -gamepad1.right_stick_x;  // +CCW

            // deadzone + shaping
            fwd  = shape(deadzone(fwd,  DEADZONE), INPUT_EXP);
            left = shape(deadzone(left, DEADZONE), INPUT_EXP);
            turn = shape(deadzone(turn, DEADZONE), INPUT_EXP);

            // slow mode (right trigger)
            double slow = lerp(1.0, SLOW_MIN, clamp01(gamepad1.right_trigger));
            fwd  *= slow;
            left *= slow;
            turn *= slow;

            // drive (Vector2d: x=forward, y=left)
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, left), turn));
            drive.updatePoseEstimate();

            // optional yaw reset
            if (gamepad1.a) {
                try { drive.lazyImu.get().resetYaw(); } catch (Exception ignored) {}
            }

            // simple telemetry
            Pose2d p = drive.localizer.getPose();
            telemetry.addData("Cmd", "f=%.2f  l=%.2f  w=%.2f", fwd, left, turn);
            telemetry.addData("Pose (in)", "x=%.2f  y=%.2f  θ=%.1f°",
                    p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
            telemetry.addData("Slow", "%.0f%%", slow * 100.0);
            telemetry.update();
        }
    }

    // helpers
    private static double deadzone(double v, double dz) { return Math.abs(v) < dz ? 0.0 : v; }
    private static double shape(double v, double exp) {
        double a = Math.abs(v);
        double out = Math.pow(a, exp);
        return Math.copySign(out, v);
    }
    private static double clamp01(double v){ return Math.max(0.0, Math.min(1.0, v)); }
    private static double lerp(double a, double b, double t){ return a + (b - a) * t; }
}
