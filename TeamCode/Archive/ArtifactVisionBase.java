package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class ArtifactVisionBase {

    // Files must live in TeamCode/src/main/assets/
    private static final String MODEL_FILE = "model.tflite";

    // If you don't have labels.txt, define them here in your model's class order:
    private static final String[] LABELS = new String[] {
            // TODO: replace with your actual classes, e.g.:
            "green", "purple", "robot"
    };

    private TfodProcessor tfodProcessor;
    private VisionPortal visionPortal;

    public ArtifactVisionBase(HardwareMap hardwareMap) {
        // Build TFOD for a custom model
        TfodProcessor.Builder tfodBuilder = new TfodProcessor.Builder()
                .setModelFileName(MODEL_FILE)
                // If you create assets/labels.txt, use .setModelLabelsFile("labels.txt") instead:
                .setModelLabels(LABELS)
                // Common model input sizes are 300/320/640 — set to what your model expects:
                .setModelInputSize(320)
                .setIsModelTensorFlow2(true)
                .setUseObjectTracker(true)
                .setNumExecutorThreads(2)
                .setMinResultConfidence(0.55f);

        tfodProcessor = tfodBuilder.build();

        // Use your USB webcam name exactly as in the RC configuration ("Webcam 1" is typical)
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tfodProcessor)
                .enableLiveView(true) // viewable from DS (⋮ → Camera Stream)
                .build();

        tfodProcessor.setActivated(true);
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }

    /** Get all recognitions this frame (may be empty). */
    public List<Recognition> getRecognitions() {
        return (tfodProcessor != null) ? tfodProcessor.getRecognitions() : null;
    }

    /** Best label by confidence or "none". */
    public String getTopLabel() {
        List<Recognition> recs = getRecognitions();
        if (recs == null || recs.isEmpty()) return "none";
        Recognition best = recs.get(0);
        for (Recognition r : recs) {
            if (r.getConfidence() > best.getConfidence()) best = r;
        }
        return best.getLabel() != null ? best.getLabel() : "none";
    }
}
