package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
// No direct TFOD imports; use reflection so code compiles even if tfod classes absent in environment.

import java.util.List;

public class ArtifactVisionBase {

    // Files must live in TeamCode/src/main/assets/
    private static final String MODEL_FILE = "model.tflite";

    // If you don't have labels.txt, define them here in your model's class order:
    private static final String[] LABELS = new String[] {
            // TODO: replace with your actual classes, e.g.:
            "green", "purple", "robot"
    };

    private Object tfodProcessor; // reflection handle (org.firstinspires.ftc.vision.tfod.TfodProcessor)
    private VisionPortal visionPortal;

    public ArtifactVisionBase(HardwareMap hardwareMap) {
        try {
            Class<?> procClazz = Class.forName("org.firstinspires.ftc.vision.tfod.TfodProcessor");
            Class<?> builderClazz = Class.forName("org.firstinspires.ftc.vision.tfod.TfodProcessor$Builder");
            Object builder = builderClazz.getConstructor().newInstance();
            builderClazz.getMethod("setModelFileName", String.class).invoke(builder, MODEL_FILE);
            builderClazz.getMethod("setModelLabels", String[].class).invoke(builder, (Object) LABELS);
            builderClazz.getMethod("setModelInputSize", int.class).invoke(builder, 320);
            builderClazz.getMethod("setIsModelTensorFlow2", boolean.class).invoke(builder, true);
            builderClazz.getMethod("setUseObjectTracker", boolean.class).invoke(builder, true);
            builderClazz.getMethod("setNumExecutorThreads", int.class).invoke(builder, 2);
            builderClazz.getMethod("setMinResultConfidence", float.class).invoke(builder, 0.55f);
            tfodProcessor = builderClazz.getMethod("build").invoke(builder);

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor((org.firstinspires.ftc.vision.VisionProcessor) tfodProcessor)
                    .enableLiveView(true)
                    .build();

            procClazz.getMethod("setActivated", boolean.class).invoke(tfodProcessor, true);
        } catch (ClassNotFoundException e) {
            // TFOD not present; leave tfodProcessor null
        } catch (Throwable t) {
            // Any other failure: null out so callers see no recognitions
            tfodProcessor = null;
        }
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }

    /** Get all recognitions this frame (may be empty). */
    @SuppressWarnings("unchecked")
    public List<?> getRecognitions() {
        if (tfodProcessor == null) return null;
        try {
            return (List<?>) tfodProcessor.getClass().getMethod("getRecognitions").invoke(tfodProcessor);
        } catch (Throwable ignored) {
            return null;
        }
    }

    /** Best label by confidence or "none". */
    public String getTopLabel() {
        List<?> recs = getRecognitions();
        if (recs == null || recs.isEmpty()) return "none";
        Object best = recs.get(0);
        float bestConf = getFloat(best, "getConfidence");
        for (Object r : recs) {
            float c = getFloat(r, "getConfidence");
            if (c > bestConf) { best = r; bestConf = c; }
        }
        String label = getString(best, "getLabel");
        return label != null ? label : "none";
    }

    // --- small reflection helpers ---
    private static float getFloat(Object o, String m) {
        try { return (Float)o.getClass().getMethod(m).invoke(o); } catch (Throwable t){ return 0f; }
    }
    private static String getString(Object o, String m) {
        try { Object v = o.getClass().getMethod(m).invoke(o); return v!=null? v.toString():null; } catch (Throwable t){ return null; }
    }
}
