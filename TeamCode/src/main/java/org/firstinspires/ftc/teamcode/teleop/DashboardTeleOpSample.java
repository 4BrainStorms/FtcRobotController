package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.telemetry.DashboardLayout;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Dashboard TeleOp Sample", group = "TeleOp")
public class DashboardTeleOpSample extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        DcMotor launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        DcMotor launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        DashboardLayout layout = new DashboardLayout(intakeMotor, launcherLeft, launcherRight, limelight);

        Pose2d pose = new Pose2d(0, 0, 0);

        telemetry.addLine("Use triggers to control intake and launcher.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.right_trigger > 0.1) {
                intakeMotor.setPower(-1.0);
                launcherLeft.setPower(0.85);
                launcherRight.setPower(0.85);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeMotor.setPower(-1.0);
                launcherLeft.setPower(0.0);
                launcherRight.setPower(0.0);
            } else {
                intakeMotor.setPower(0.0);
                launcherLeft.setPower(0.0);
                launcherRight.setPower(0.0);
            }

            layout.stream(pose);
            sleep(20);
        }
    }
}