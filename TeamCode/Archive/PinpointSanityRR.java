package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.util.StartPoses;

@TeleOp(name = "Pinpoint Sanity (RR)", group = "test")
public class PinpointSanityRR extends LinearOpMode {

    @Override
    public void runOpMode() {
        PinpointLocalizer loc = new PinpointLocalizer(hardwareMap, StartPoses.START_D1);

        telemetry.addLine("A=resetPos+IMU, B=recal IMU, X=zero XY, Dpad Up/Down: yawScalar ±0.01");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean aPrev=false, bPrev=false, xPrev=false, upPrev=false, dnPrev=false;

        while (opModeIsActive()) {
            // read once per loop
            loc.driver.update();

            // Pose & velocity from Pinpoint
            Pose2D p = loc.driver.getPosition();
            double xIn = p.getX(DistanceUnit.MM) / 25.4;
            double yIn = p.getY(DistanceUnit.MM) / 25.4;
            double hDeg = p.getHeading(AngleUnit.DEGREES);

            double vxIn = loc.driver.getVelX(DistanceUnit.MM) / 25.4;
            double vyIn = loc.driver.getVelY(DistanceUnit.MM) / 25.4;
            double wDeg = loc.driver.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            // Buttons
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean up = gamepad1.dpad_up;
            boolean dn = gamepad1.dpad_down;

            if (a && !aPrev)  loc.driver.resetPosAndIMU(); // zero pose + IMU recal
            if (b && !bPrev)  loc.driver.recalibrateIMU(); // IMU recal only
            if (x && !xPrev) {
                // zero XY, keep current heading
                loc.driver.setPosition(
                        new Pose2D(
                                DistanceUnit.MM,                // units for x/y
                                0, 0,                           // x, y
                                AngleUnit.RADIANS,              // units for heading
                                p.getHeading(AngleUnit.RADIANS) // keep current heading
                        )
                );
            }
            if (up && !upPrev) loc.driver.setYawScalar(loc.driver.getYawScalar() + 0.01);
            if (dn && !dnPrev) loc.driver.setYawScalar(loc.driver.getYawScalar() - 0.01);

            aPrev=a; bPrev=b; xPrev=x; upPrev=up; dnPrev=dn;

            telemetry.addData("Status", loc.driver.getDeviceStatus());
            telemetry.addData("Pinpoint Hz", "%.1f", loc.driver.getFrequency());
            telemetry.addData("Pose (in,deg)", "x=%.2f  y=%.2f  h=%.1f", xIn, yIn, hDeg);
            telemetry.addData("Vel  (in/s,deg/s)", "x=%.2f  y=%.2f  w=%.1f", vxIn, vyIn, wDeg);
            telemetry.addData("YawScalar", "%.3f", loc.driver.getYawScalar());
            telemetry.addLine("Sanity: push FORWARD -> X increases; push LEFT -> Y increases.");
            telemetry.update();

            sleep(10);
        }
    }
}
