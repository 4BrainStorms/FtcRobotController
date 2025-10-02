package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.ArrayList;
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

    /**
     * Parse a full RRPathGen export string.
     * Supports:
     *  - new Pose2d(x, y, Math.toRadians(h))
     *  - splineTo(new Vector2d(x,y), Math.toRadians(h))
     *  - splineToLinearHeading(new Pose2d(x,y,Math.toRadians(h)), Math.toRadians(h))
     * @param exportTxt the exact text you copy from RRPathGen (left panel)
     * @param frame RRPathGen coordinate frame (CENTER or CORNER)
     * @param startFromD1 if true, ignore the RRPathGen start and begin at StartPoses.START_D1
     */
    public static Pose2d[] fromRRPathGen(String exportTxt, Frame frame, boolean startFromD1) {
        ArrayList<Pose2d> pts = new ArrayList<>();

        // (1) start pose: new Pose2d(x, y, Math.toRadians(h))
        Pattern pStart = Pattern.compile("new\\s+Pose2d\\s*\\(\\s*([-\\d.]+)\\s*,\\s*([-\\d.]+)\\s*,\\s*Math\\.toRadians\\(\\s*([-\\d.]+)\\s*\\)\\s*\\)");
        Matcher mStart = pStart.matcher(exportTxt);
        Pose2d start = null;
        if (mStart.find()) {
            double x = Double.parseDouble(mStart.group(1));
            double y = Double.parseDouble(mStart.group(2));
            double h = Math.toRadians(Double.parseDouble(mStart.group(3)));
            start = toCorner(new Pose2d(x, y, h), frame);
            if (!startFromD1) pts.add(start);
        }

        // (2) splineTo(Vector2d(x,y), Math.toRadians(h))
        Pattern pVec = Pattern.compile("new\\s+Vector2d\\s*\\(\\s*([-\\d.]+)\\s*,\\s*([-\\d.]+)\\s*\\)\\s*,\\s*Math\\.toRadians\\(\\s*([-\\d.]+)\\s*\\)");
        Matcher mVec = pVec.matcher(exportTxt);
        while (mVec.find()) {
            double x = Double.parseDouble(mVec.group(1));
            double y = Double.parseDouble(mVec.group(2));
            double h = Math.toRadians(Double.parseDouble(mVec.group(3)));
            pts.add(toCorner(new Pose2d(x, y, h), frame));
        }

        // (3) splineToLinearHeading(new Pose2d(x,y,Math.toRadians(h)), Math.toRadians(h2))
        Pattern pPose = Pattern.compile("splineToLinearHeading\\s*\\(\\s*new\\s+Pose2d\\s*\\(\\s*([-\\d.]+)\\s*,\\s*([-\\d.]+)\\s*,\\s*Math\\.toRadians\\(\\s*([-\\d.]+)\\s*\\)\\s*\\)");
        Matcher mPose = pPose.matcher(exportTxt);
        while (mPose.find()) {
            double x = Double.parseDouble(mPose.group(1));
            double y = Double.parseDouble(mPose.group(2));
            double h = Math.toRadians(Double.parseDouble(mPose.group(3)));
            pts.add(toCorner(new Pose2d(x, y, h), frame));
        }

        // Prepend D1 if requested
        if (startFromD1) {
            ArrayList<Pose2d> withStart = new ArrayList<>();
            withStart.add(StartPoses.START_D1);
            withStart.addAll(pts);
            return withStart.toArray(new Pose2d[0]);
        }

        // Otherwise ensure we included the RR start at index 0
        if (pts.isEmpty() && start != null) pts.add(start);
        return pts.toArray(new Pose2d[0]);
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
}
