package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="LL3A USB Sanity2 (FPS)", group="Vision")
public class LLUsbSanity extends OpMode {
    private Limelight3A ll;
    private int currentPipe = 0;

    // simple FPS estimator
    private long fpsT0;
    private int nonNullCount = 0, validCount = 0;
    private double nonNullFps = 0, validFps = 0;

    // edge detect for D-pad
    private boolean upWas, leftWas, rightWas, downWas;

    @Override public void init() {
        ll = hardwareMap.get(Limelight3A.class, "limelight"); // device name in Robot Config
        try { ll.pipelineSwitch(currentPipe); } catch (Exception ignored) {}
        fpsT0 = System.currentTimeMillis();

        telemetry.addLine("Aiming at an AprilTag?");
        telemetry.addLine("D-pad UP/LEFT/RIGHT/DOWN = pipelines 0/1/2/3");
        telemetry.addLine("Press PLAY to start the LL USB stream.");
        telemetry.update();
    }

    @Override public void start() {
        try { ll.start(); } catch (Exception ignored) {}
        // (optional) re-assert pipeline after start
        try { ll.pipelineSwitch(currentPipe); } catch (Exception ignored) {}
    }

    @Override public void loop() {
        // pipeline switching with edge-detected D-pad
        if (gamepad1.dpad_up   && !upWas)   { currentPipe = 0; try { ll.pipelineSwitch(0); } catch (Exception ignored) {} }
        if (gamepad1.dpad_left && !leftWas) { currentPipe = 1; try { ll.pipelineSwitch(1); } catch (Exception ignored) {} }
        if (gamepad1.dpad_right&& !rightWas){ currentPipe = 2; try { ll.pipelineSwitch(2); } catch (Exception ignored) {} }
        if (gamepad1.dpad_down && !downWas) { currentPipe = 3; try { ll.pipelineSwitch(3); } catch (Exception ignored) {} }
        upWas   = gamepad1.dpad_up;
        leftWas = gamepad1.dpad_left;
        rightWas= gamepad1.dpad_right;
        downWas = gamepad1.dpad_down;

        LLResult r = ll.getLatestResult();
        if (r != null) {
            nonNullCount++;
            telemetry.addData("valid", r.isValid());
            telemetry.addData("tx", r.getTx());
            telemetry.addData("ty", r.getTy());
            telemetry.addData("ta", r.getTa());

            if (r.getFiducialResults() != null) {
                StringBuilder ids = new StringBuilder();
                for (FiducialResult f : r.getFiducialResults()) ids.append(f.getFiducialId()).append(" ");
                telemetry.addData("IDs", ids.toString());
            } else telemetry.addData("IDs", "(none)");

            if (r.isValid()) validCount++;
        } else {
            telemetry.addLine("No LL result yet");
        }

        long now = System.currentTimeMillis();
        if (now - fpsT0 >= 1000) {
            nonNullFps = nonNullCount * 1000.0 / (now - fpsT0);
            validFps   = validCount   * 1000.0 / (now - fpsT0);
            nonNullCount = 0; validCount = 0; fpsT0 = now;
        }

        // ONE LINE: frames are flowing
        telemetry.addData("LL frames", "pipe=%d  nonNull=%.1f fps  valid=%.1f fps", currentPipe, nonNullFps, validFps);
        telemetry.update();
    }

    @Override public void stop() {
        try { ll.stop(); } catch (Exception ignored) {}
    }
}
