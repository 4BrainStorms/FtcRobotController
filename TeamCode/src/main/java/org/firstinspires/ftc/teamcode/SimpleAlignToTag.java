package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

@TeleOp(name = "Simple Align to Tag", group = "drive")
public class SimpleAlignToTag extends LinearOpMode {

    private static final double ALIGN_TOLERANCE_DEG = 3.0;
    private static final double ALIGN_TURN_SPEED = 0.15;

    @Override
    public void runOpMode() {
        // Provide initial pose estimate (x=0, y=0, heading=0)
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();
            boolean visible = result != null && result.isValid();

            double fwd = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            if (gamepad1.y && visible) {
                double tx = result.getTx(); // Horizontal offset in degrees
                if (Math.abs(tx) > ALIGN_TOLERANCE_DEG) {
                    turn = (tx > 0) ? -ALIGN_TURN_SPEED : ALIGN_TURN_SPEED;
                    fwd = 0.0;
                    strafe = 0.0;
                } else {
                    turn = 0.0; // Aligned
                }
            }

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, strafe), turn));
            drive.updatePoseEstimate();

            telemetry.addData("Tag Visible", visible);
            telemetry.addData("tx", visible ? result.getTx() : "N/A");
            telemetry.addData("Turn Cmd", turn);
            telemetry.update();
        }
    }
}
