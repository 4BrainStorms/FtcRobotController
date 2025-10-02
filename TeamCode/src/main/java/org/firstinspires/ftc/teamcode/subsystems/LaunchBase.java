package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import java.util.List;

public class LaunchBase {
    private DcMotor launcherLeft;
    private DcMotor launcherRight;

    public LaunchBase(HardwareMap hardwareMap) {
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void launch(double power) {
        launcherLeft.setPower(power);
        launcherRight.setPower(power);
    }

    public void stop() {
        launcherLeft.setPower(0.0);
        launcherRight.setPower(0.0);
    }

    public boolean alignToTag(List<FiducialResult> fiducials) {
        if (fiducials == null || fiducials.isEmpty()) return false;
        FiducialResult tag = fiducials.get(0);
        double tx = tag.getTargetXDegrees();
        return Math.abs(tx) < 2.0;
    }
}