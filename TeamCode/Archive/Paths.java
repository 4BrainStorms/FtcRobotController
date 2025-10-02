package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Helpers to turn RRPathGen exports or simple x,y,deg lists into Pose2d[]. */
public final class Paths {
    private Paths() {}
    public enum Frame { CENTER, CORNER } // RRPathGen is usually CENTER-origin (0,0 at field center)

    private static final double HALF_FIELD_IN = 72.0;

    private static Pose2d toCorner(Pose2d p, Frame frame) {
        if (frame == Frame.CORNER) return p;
        return new Pose2d(p.position.x + HALF_FIELD_IN, p.position.y + HALF_FIELD_IN, p.heading.toDouble());
    }

    /** Simple: type points as {x,y,deg} (inches & degrees). */
    public static Pose2d[] fromDeg(double[][] points, Frame frame, boolean startFromD1) {
        ArrayList<Pose2d> out = new ArrayList<>();
        if (startFromD1) out.add(StartPoses.START_D1);
        for (double[] p : points) {
            Pose2d prr = new Pose2d(p[0], p[1], Math.toRadians(p[2]));
            out.add(toCorner(prr, frame));
        }
        return out.toArray(new Pose2d[0]);
    }

    /**
     * Parse a full RRPathGen export string.
     * Supports (order preserved):
     *  - splineTo(Vector2d, tangentDeg)
     *  - splineToConstantHeading(Vector2d, [tangentDeg])
     *  - splineToLinearHeading(Pose2d, [tangentDeg])
     *  - splineToSplineHeading(Pose2d, [tangentDeg])
     *  - lineTo(Vector2d)
     *  - lineToConstantHeading(Vector2d)
     *  - lineToLinearHeading(Pose2d)
     *  - lineToSplineHeading(Pose2d)
     *  - strafeTo(Vector2d)
     *  - strafeToConstantHeading(Vector2d)
     */
    public static Pose2d[] fromRRPathGen(String exportTxt, Frame frame, boolean startFromD1) {
        class Op {
            int idx;
            String opName;     // e.g., splineTo, lineToLinearHeading, strafeTo, ...
            double x, y;       // (center-origin inches)
            Double headingDeg; // for Pose2d-based ops (end robot heading)
            Double tangentDeg; // optional tangent for some spline ops
        }
        ArrayList<Op> ops = new ArrayList<>();

        // --- start pose ---
        Pattern pStart = Pattern.compile(
                "new\\s+Pose2d\\s*\\(\\s*([-\\d.]+)\\s*,\\s*([-\\d.]+)\\s*,\\s*Math\\.toRadians\\(\\s*([-\\d.]+)\\s*\\)\\s*\\)");
        Matcher mStart = pStart.matcher(exportTxt);
        Pose2d exportStart = null;
        if (mStart.find()) {
            double x = Double.parseDouble(mStart.group(1));
            double y = Double.parseDouble(mStart.group(2));
            double h = Math.toRadians(Double.parseDouble(mStart.group(3)));
            exportStart = toCorner(new Pose2d(x, y, h), frame);
        }

        // --- Pose2d-based ops (heading given by Pose2d) ---
        Pattern pPoseOps = Pattern.compile(
                "(splineToLinearHeading|splineToSplineHeading|lineToLinearHeading|lineToSplineHeading|strafeToLinearHeading|strafeToSplineHeading)" +
                        "\\s*\\(\\s*new\\s+Pose2d\\s*\\(\\s*([-\\d.]+)\\s*,\\s*([-\\d.]+)\\s*,\\s*Math\\.toRadians\\(\\s*([-\\d.]+)\\s*\\)\\s*\\)" +
                        "\\s*(?:,\\s*Math\\.toRadians\\([^)]*\\)\\s*)?\\)");
        Matcher mPoseOps = pPoseOps.matcher(exportTxt);
        while (mPoseOps.find()) {
            Op o = new Op();
            o.idx = mPoseOps.start();
            o.opName = mPoseOps.group(1);
            o.x = Double.parseDouble(mPoseOps.group(2));
            o.y = Double.parseDouble(mPoseOps.group(3));
            o.headingDeg = Double.parseDouble(mPoseOps.group(4)); // end robot heading
            ops.add(o);
        }

        // --- Vector2d-based ops ---
        // Includes: splineTo, splineToConstantHeading, lineTo, lineToConstantHeading, strafeTo, strafeToConstantHeading
        Pattern pVecOps = Pattern.compile(
                "(splineTo|splineToConstantHeading|lineTo|lineToConstantHeading|strafeTo|strafeToConstantHeading)" +
                        "\\s*\\(\\s*new\\s+Vector2d\\s*\\(\\s*([-\\d.]+)\\s*,\\s*([-\\d.]+)\\s*\\)" +
                        "\\s*(?:,\\s*Math\\.toRadians\\(\\s*([-\\d.]+)\\s*\\)\\s*)?\\)");
        Matcher mVecOps = pVecOps.matcher(exportTxt);
        while (mVecOps.find()) {
            Op o = new Op();
            o.idx = mVecOps.start();
            o.opName = mVecOps.group(1);
            o.x = Double.parseDouble(mVecOps.group(2));
            o.y = Double.parseDouble(mVecOps.group(3));
            if (mVecOps.group(4) != null) o.tangentDeg = Double.parseDouble(mVecOps.group(4));
            ops.add(o);
        }

        // Preserve export order
        Collections.sort(ops, Comparator.comparingInt(a -> a.idx));

        // Build output, computing headings according to RR semantics
        ArrayList<Pose2d> out = new ArrayList<>();
        double lastHeading = startFromD1
                ? StartPoses.START_D1.heading.toDouble()
                : (exportStart != null ? exportStart.heading.toDouble() : 0.0);

        if (startFromD1) {
            out.add(StartPoses.START_D1);
        } else if (exportStart != null) {
            out.add(exportStart);
        }

        for (Op o : ops) {
            double endHeadingRad;
            if (o.headingDeg != null) {
                // Pose2d-based ops set the end heading explicitly
                endHeadingRad = Math.toRadians(o.headingDeg);
            } else {
                // Vector2d-based ops
                switch (o.opName) {
                    case "splineTo":
                        endHeadingRad = (o.tangentDeg != null)
                                ? Math.toRadians(o.tangentDeg)
                                : lastHeading;
                        break;
                    case "splineToConstantHeading":
                    case "lineTo":
                    case "lineToConstantHeading":
                    case "strafeTo":
                    case "strafeToConstantHeading":
                    default:
                        endHeadingRad = lastHeading; // keep previous heading
                        break;
                }
            }
            Pose2d pCorner = toCorner(new Pose2d(o.x, o.y, endHeadingRad), frame);
            out.add(pCorner);
            lastHeading = endHeadingRad;
        }

        if (out.isEmpty() && exportStart != null) out.add(exportStart);
        return out.toArray(new Pose2d[0]);
    }
}
