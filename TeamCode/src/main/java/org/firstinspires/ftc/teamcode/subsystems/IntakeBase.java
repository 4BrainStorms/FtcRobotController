package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeBase {
    private DcMotor intakeMotor;

    public IntakeBase(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intakeOn() {
        intakeMotor.setPower(-1.0);
    }

    public void intakeOff() {
        intakeMotor.setPower(0.0);
    }
}