package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimelightBase {
    private Limelight3A limelight;

    public LimelightBase(HardwareMap hardwareMap, String name) {
        limelight = hardwareMap.get(Limelight3A.class, name);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public List<FiducialResult> getFiducials() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getFiducialResults();
        }
        return null;
    }

    public void switchPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    public void stop() {
        limelight.stop();
    }
}