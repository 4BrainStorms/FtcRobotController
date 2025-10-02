package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

@TeleOp(name = "Dual Controller Align", group = "drive")
public class DualControllerAlign extends LinearOpMode {

    private static final double ALIGN_TOLERANCE_DEG = 3.0;
    private static final double ALIGN_TURN_SPEED = 0.15;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor launcher = hardwareMap.get(DcMotor.class, "launcher");

        limelight.setPollRateHz(100);
        limelight.start();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();
            boolean visible = result != null && result.isValid();

            // === Driver Controls (gamepad1) ===
            double fwd = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            if (gamepad1.y && visible) {
                double tx = result.getTx();
                if (Math.abs(tx) > ALIGN_TOLERANCE_DEG) {
                    turn = (tx > 0) ? -ALIGN_TURN_SPEED : ALIGN_TURN_SPEED;
                    fwd = 0.0;
                    strafe = 0.0;
                } else {
                    turn = 0.0;
                }
            }

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, strafe), turn));
            drive.updatePoseEstimate();

            // === Operator Controls (gamepad2) ===
            if (gamepad2.left_trigger > 0.1) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }

            if (gamepad2.right_trigger > 0.1) {
                launcher.setPower(1.0);
            } else {
                launcher.setPower(0.0);
            }

            telemetry.addData("Tag Visible", visible);
            telemetry.addData("tx", visible ? result.getTx() : "N/A");
            telemetry.addData("Turn Cmd", turn);
            telemetry.update();
        }
    }
}