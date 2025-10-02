package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Autonomous(name = "RRPathGen → Actions (Center Origin, with conversion)", group = "auton")
public class RRPG_ActionsRun_Center extends LinearOpMode {

    // === Paste your RRPathGen export EXACTLY as copied (CENTER-origin, inches) ===
    private static final String RR_EXPORT =
            "TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-0.40, 0.20, Math.toRadians(90.00)))\n" +
                    ".lineToLinearHeading(new Pose2d(23.67, 0.20, Math.toRadians(0.00)))\n" +
                    ".lineToLinearHeading(new Pose2d(23.87, 24.27, Math.toRadians(89.53)))\n" +
                    ".lineToLinearHeading(new Pose2d(0.00, 24.27, Math.toRadians(180.00)))\n" +
                    ".lineToLinearHeading(new Pose2d(-0.40, 0.40, Math.toRadians(269.41)))\n" +
                    ".build();\n";
    // ============================================================================

    // ----------------- Frame conversion: FIELD (MeepMeep) -> Road Runner -----------------
    private static double angFieldToRR(double rad) { return rad - Math.PI / 2.0; } // -90°
    private static Vector2d vecFieldToRR(double xField, double yField) { return new Vector2d(yField, -xField); }
    private static Pose2d poseFieldToRR(Pose2d pField) {
        return new Pose2d(pField.position.y, -pField.position.x, angFieldToRR(pField.heading.toDouble()));
    }
    // -------------------------------------------------------------------------------------

    @Override
    public void runOpMode() {
        Pose2d startField = parseStartPose(RR_EXPORT);           // FIELD frame
        List<Seg> segs = parseSegmentsPreserveOrder(RR_EXPORT);  // FIELD frame

        if (startField == null || segs.isEmpty()) {
            telemetry.addLine("Parse error: RR_EXPORT is empty or malformed.");
            telemetry.update();
            return;
        }

        Pose2d start = poseFieldToRR(startField); // convert to RR frame

        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        // Optional gentle gains
        MecanumDrive.PARAMS.axialGain   = 2.5;
        MecanumDrive.PARAMS.lateralGain = 2.5;
        MecanumDrive.PARAMS.headingGain = 10.0;

        TrajectoryActionBuilder b = drive.actionBuilder(start);
        Pose2d cursor = start;
        double endHeadingRad = cursor.heading.toDouble();

        // Build trajectory in RR frame
        for (Seg s : segs) {
            Vector2d t = vecFieldToRR(s.x, s.y);
            double tx = t.x, ty = t.y;

            double tanRR = Double.isNaN(s.tanRad)
                    ? Math.atan2(ty - cursor.position.y, tx - cursor.position.x)
                    : angFieldToRR(s.tanRad);
            double headRR = angFieldToRR(s.headRad);

            switch (s.type) {
                case SPLINE_TO:
                    b = b.splineTo(new Vector2d(tx, ty), tanRR);
                    endHeadingRad = tanRR;
                    cursor = new Pose2d(tx, ty, endHeadingRad);
                    break;
                case SPLINE_TO_CONSTANT_HEADING:
                    b = b.splineToConstantHeading(new Vector2d(tx, ty), tanRR);
                    cursor = new Pose2d(tx, ty, endHeadingRad);
                    break;
                case SPLINE_TO_LINEAR_HEADING:
                    b = b.splineToLinearHeading(new Pose2d(tx, ty, headRR), tanRR);
                    endHeadingRad = headRR;
                    cursor = new Pose2d(tx, ty, endHeadingRad);
                    break;
                case SPLINE_TO_SPLINE_HEADING:
                    b = b.splineToSplineHeading(new Pose2d(tx, ty, headRR), tanRR);
                    endHeadingRad = headRR;
                    cursor = new Pose2d(tx, ty, endHeadingRad);
                    break;
                case LINE_TO:
                    b = b.splineTo(new Vector2d(tx, ty), tanRR);
                    endHeadingRad = tanRR;
                    cursor = new Pose2d(tx, ty, endHeadingRad);
                    break;
                case LINE_TO_CONSTANT_HEADING:
                    b = b.splineToConstantHeading(new Vector2d(tx, ty), tanRR);
                    cursor = new Pose2d(tx, ty, endHeadingRad);
                    break;
                case LINE_TO_LINEAR_HEADING:
                    b = b.splineToLinearHeading(new Pose2d(tx, ty, headRR), tanRR);
                    endHeadingRad = headRR;
                    cursor = new Pose2d(tx, ty, endHeadingRad);
                    break;
                case LINE_TO_SPLINE_HEADING:
                    b = b.splineToSplineHeading(new Pose2d(tx, ty, headRR), tanRR);
                    endHeadingRad = headRR;
                    cursor = new Pose2d(tx, ty, endHeadingRad);
                    break;
                case TURN:
                    b = b.turn(s.turnRad); // robot-relative
                    endHeadingRad += s.turnRad;
                    cursor = new Pose2d(cursor.position.x, cursor.position.y, endHeadingRad);
                    break;
            }
        }

        Action action = b.build();

        telemetry.addLine("RRPathGen path (CENTER origin) ready");
        telemetry.addData("Start (FIELD)", "x=%.2f y=%.2f h=%.1f°",
                startField.position.x, startField.position.y, Math.toDegrees(startField.heading.toDouble()));
        telemetry.addData("Start  (RR)  ", "x=%.2f y=%.2f h=%.1f°",
                start.position.x, start.position.y, Math.toDegrees(start.heading.toDouble()));
        telemetry.addData("End    (RR)  ", "x=%.2f y=%.2f h=%.1f°",
                cursor.position.x, cursor.position.y, Math.toDegrees(endHeadingRad));
        telemetry.addData("# segments", segs.size());
        telemetry.update();

        while (opModeInInit()) {
            Pose2d pp = drive.localizer.getPose();
            telemetry.addData("Pinpoint NOW (RR)", "x=%.2f y=%.2f h=%.1f°",
                    pp.position.x, pp.position.y, Math.toDegrees(pp.heading.toDouble()));
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(action);
    }

    // ----------------- Center-origin parser (order-preserving, line-by-line) -----------------
    private static final Pattern START_POSE = Pattern.compile(
            "new\\s+Pose2d\\(\\s*([-0-9.]+)\\s*,\\s*([-0-9.]+)\\s*,\\s*Math\\.toRadians\\(\\s*([-0-9.]+)\\s*\\)\\s*\\)");
    private static final Pattern P_VEC   = Pattern.compile("new\\s+Vector2d\\(\\s*([-0-9.]+)\\s*,\\s*([-0-9.]+)\\s*\\)");
    private static final Pattern P_POSE  = Pattern.compile("new\\s+Pose2d\\(\\s*([-0-9.]+)\\s*,\\s*([-0-9.]+)\\s*,\\s*Math\\.toRadians\\(\\s*([-0-9.]+)\\s*\\)\\s*\\)");
    private static final Pattern P_RAD   = Pattern.compile("Math\\.toRadians\\(\\s*([-0-9.]+)\\s*\\)");

    private static Pose2d parseStartPose(String s) {
        Matcher m = START_POSE.matcher(s);
        if (!m.find()) return null;
        double x = Double.parseDouble(m.group(1));
        double y = Double.parseDouble(m.group(2));
        double hDeg = Double.parseDouble(m.group(3));
        return new Pose2d(x, y, Math.toRadians(hDeg));
    }

    private static List<Seg> parseSegmentsPreserveOrder(String s) {
        List<Seg> out = new ArrayList<>();
        String[] lines = s.split("\\r?\\n");
        for (String raw : lines) {
            String line = raw.trim();
            if (line.isEmpty()) continue;

            if (line.contains(".splineToSplineHeading(")) {
                double[] p = readPose(line);
                double tan = readRad(line, 2);
                out.add(Seg.splineToSpline(p[0], p[1], p[2], tan));

            } else if (line.contains(".splineToLinearHeading(")) {
                double[] p = readPose(line);
                double tan = readRad(line, 2);
                out.add(Seg.splineToLinear(p[0], p[1], p[2], tan));

            } else if (line.contains(".splineToConstantHeading(")) {
                double[] v = readVec(line);
                double tan = readRad(line, 1);
                out.add(Seg.splineToConst(v[0], v[1], tan));

            } else if (line.contains(".splineTo(")) {
                double[] v = readVec(line);
                double tan = readRad(line, 1);
                out.add(Seg.splineTo(v[0], v[1], tan));

            } else if (line.contains(".lineToSplineHeading(")) {
                double[] p = readPose(line);
                out.add(Seg.lineToSpline(p[0], p[1], p[2]));

            } else if (line.contains(".lineToLinearHeading(")) {
                double[] p = readPose(line);
                out.add(Seg.lineToLinear(p[0], p[1], p[2]));

            } else if (line.contains(".lineToConstantHeading(")) {
                double[] v = readVec(line);
                out.add(Seg.lineToConst(v[0], v[1]));

            } else if (line.contains(".lineTo(")) {
                double[] v = readVec(line);
                Double tanMaybe = tryReadRad(line);
                out.add(Seg.lineTo(v[0], v[1], tanMaybe == null ? Double.NaN : tanMaybe));

            } else if (line.contains(".turn(")) {
                double ang = readRad(line, 1);
                out.add(Seg.turn(ang));
            }
        }
        return out;
    }

    // --- tiny parse helpers ---
    private static double[] readVec(String line) {
        Matcher m = P_VEC.matcher(line);
        if (!m.find()) throw new IllegalArgumentException("Vector2d not found: " + line);
        return new double[]{ Double.parseDouble(m.group(1)), Double.parseDouble(m.group(2)) };
    }
    private static double[] readPose(String line) {
        Matcher m = P_POSE.matcher(line);
        if (!m.find()) throw new IllegalArgumentException("Pose2d not found: " + line);
        return new double[]{
                Double.parseDouble(m.group(1)),
                Double.parseDouble(m.group(2)),
                Math.toRadians(Double.parseDouble(m.group(3)))
        };
    }
    private static double readRad(String line, int nth) { // 1-based
        Matcher m = P_RAD.matcher(line);
        int i = 0;
        while (m.find()) {
            i++;
            if (i == nth) return Math.toRadians(Double.parseDouble(m.group(1)));
        }
        throw new IllegalArgumentException("Radians arg #" + nth + " not found: " + line);
    }
    private static Double tryReadRad(String line) {
        Matcher m = P_RAD.matcher(line);
        if (m.find()) return Math.toRadians(Double.parseDouble(m.group(1)));
        return null;
    }

    // ----- segment model -----
    private static class Seg {
        enum Type {
            SPLINE_TO, SPLINE_TO_CONSTANT_HEADING, SPLINE_TO_LINEAR_HEADING, SPLINE_TO_SPLINE_HEADING,
            LINE_TO, LINE_TO_CONSTANT_HEADING, LINE_TO_LINEAR_HEADING, LINE_TO_SPLINE_HEADING,
            TURN
        }
        final Type type;
        final double x, y, headRad, tanRad, turnRad; // FIELD frame values (radians)
        private Seg(Type t, double x, double y, double headRad, double tanRad, double turnRad) {
            this.type=t; this.x=x; this.y=y; this.headRad=headRad; this.tanRad=tanRad; this.turnRad=turnRad;
        }
        static Seg splineTo(double x, double y, double tan){ return new Seg(Type.SPLINE_TO, x,y, 0, tan, 0); }
        static Seg splineToConst(double x, double y, double tan){ return new Seg(Type.SPLINE_TO_CONSTANT_HEADING, x,y, 0, tan, 0); }
        static Seg splineToLinear(double x, double y, double head, double tan){ return new Seg(Type.SPLINE_TO_LINEAR_HEADING, x,y, head, tan, 0); }
        static Seg splineToSpline(double x, double y, double head, double tan){ return new Seg(Type.SPLINE_TO_SPLINE_HEADING, x,y, head, tan, 0); }
        static Seg lineTo(double x, double y, double tanOrNaN){ return new Seg(Type.LINE_TO, x,y, 0, tanOrNaN, 0); }
        static Seg lineToConst(double x, double y){ return new Seg(Type.LINE_TO_CONSTANT_HEADING, x,y, 0, 0, 0); }
        static Seg lineToLinear(double x, double y, double head){ return new Seg(Type.LINE_TO_LINEAR_HEADING, x,y, head, 0, 0); }
        static Seg lineToSpline(double x, double y, double head){ return new Seg(Type.LINE_TO_SPLINE_HEADING, x,y, head, 0, 0); }
        static Seg turn(double ang){ return new Seg(Type.TURN, Double.NaN, Double.NaN, 0, 0, ang); }
    }
}
