package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.telemetry.DashboardLayout;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Autonomous(name = "Dashboard Auton Sample", group = "Auton")
public class DashboardAutonSample extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        DcMotor launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        DcMotor launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        DashboardLayout layout = new DashboardLayout(intakeMotor, launcherLeft, launcherRight, limelight);

        Pose2d pose = new Pose2d(0, 0, 0);

        telemetry.addLine("Ready to launch with dashboard telemetry.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Simulate launch sequence
        launcherLeft.setPower(0.85);
        launcherRight.setPower(0.85);
        intakeMotor.setPower(-1.0);

        for (int i = 0; i < 100; i++) {
            layout.stream(pose);
            sleep(20);
        }

        launcherLeft.setPower(0);
        launcherRight.setPower(0);
        intakeMotor.setPower(0);
    }
}