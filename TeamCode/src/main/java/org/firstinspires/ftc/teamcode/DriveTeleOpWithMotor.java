package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

@TeleOp(name = "Drive TeleOp with Intake + Launcher", group = "drive")
public class DriveTeleOpWithMotor extends LinearOpMode {

    private static final double DEADZONE = 0.05;
    private static final double INPUT_EXP = 2.0;
    private static final double SLOW_MIN = 0.35;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Initialize motors
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        DcMotor launcherMotor = hardwareMap.get(DcMotor.class, "launcher");

        telemetry.addLine("Robot-centric drive + intake/launcher. RT = intake+launcher, LT = intake only, A = reset yaw.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double fwd  = -gamepad1.left_stick_y;
            double left = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            fwd  = shape(deadzone(fwd,  DEADZONE), INPUT_EXP);
            left = shape(deadzone(left, DEADZONE), INPUT_EXP);
            turn = shape(deadzone(turn, DEADZONE), INPUT_EXP);

            double slow = lerp(1.0, SLOW_MIN, clamp01(gamepad1.right_trigger));
            fwd  *= slow;
            left *= slow;
            turn *= slow;

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, left), turn));
            drive.updatePoseEstimate();

            // Motor control logic
            if (gamepad1.right_trigger > 0.1) {
                intakeMotor.setPower(0.30);       // intake forward port 0
                launcherMotor.setPower(1.00);    // launcher reverse port 1
            } else if (gamepad1.left_trigger > 0.1) {
                intakeMotor.setPower(1.0);      // intake forward
                launcherMotor.setPower(0.0);     // launcher off
            } else {
                intakeMotor.setPower(0.0);
                launcherMotor.setPower(0.0);
            }

            if (gamepad1.a) {
                try { drive.lazyImu.get().resetYaw(); } catch (Exception ignored) {}
            }

            Pose2d p = drive.localizer.getPose();
            telemetry.addData("Cmd", "f=%.2f  l=%.2f  w=%.2f", fwd, left, turn);
            telemetry.addData("Pose (in)", "x=%.2f  y=%.2f  θ=%.1f°",
                    p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
            telemetry.addData("Slow", "%.0f%%", slow * 100.0);
            telemetry.addData("Motor", "LT=%.2f RT=%.2f Intake=%.2f Launcher=%.2f",
                    gamepad1.left_trigger, gamepad1.right_trigger,
                    intakeMotor.getPower(), launcherMotor.getPower());
            telemetry.update();
        }
    }

    // Helper methods
    private static double deadzone(double v, double dz) {
        return Math.abs(v) < dz ? 0.0 : v;
    }

    private static double shape(double v, double exp) {
        double a = Math.abs(v);
        double out = Math.pow(a, exp);
        return Math.copySign(out, v);
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
