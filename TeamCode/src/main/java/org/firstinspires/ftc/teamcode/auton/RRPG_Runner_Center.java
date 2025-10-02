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
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Autonomous(name = "RRPathGen Runner (Center origin, Pinpoint)", group = "auton")
public class RRPG_Runner_Center extends LinearOpMode {

    // === Paste your RRPathGen export EXACTLY as copied (center-origin, inches) ===
    private static final String RR_EXPORT =
            "TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(12.12, -71.17, Math.toRadians(90.00)))\n" +
                    ".splineTo(new Vector2d(35.53, -50.24), Math.toRadians(51.94))\n" +
                    ".splineTo(new Vector2d(37.81, -24.55), Math.toRadians(61.14))\n" +
                    ".lineTo(new Vector2d(39.68, 14.40))\n" +
                    ".splineToSplineHeading(new Pose2d(16.68, 32.43, Math.toRadians(141.91)), Math.toRadians(141.91))\n" +
                    ".build();\n";
    // ============================================================================

    // -------- FIELD (MeepMeep/RRPathGen) → Road Runner conversion --------
    // Field: X right, Y up, 0°=+X ; RR: x forward, y left, 0=+x
    private static double angFieldToRR(double rad) { return rad - Math.PI/2.0; } // -90°
    private static Vector2d vecFieldToRR(double xF, double yF) { return new Vector2d(yF, -xF); }
    private static Pose2d poseFieldToRR(Pose2d pF) {
        return new Pose2d(pF.position.y, -pF.position.x, angFieldToRR(pF.heading.toDouble()));
    }
    // ---------------------------------------------------------------------

    @Override
    public void runOpMode() {
        Pose2d startField = parseStartPose(RR_EXPORT);                // FIELD frame
        List<Seg> segsField = parseSegmentsPreserveOrder(RR_EXPORT);  // FIELD frame

        if (startField == null || segsField.isEmpty()) {
            telemetry.addLine("Parse error: RR_EXPORT empty or malformed.");
            telemetry.addLine("Ensure it starts with trajectorySequenceBuilder(new Pose2d(..., Math.toRadians(...))).");
            telemetry.update();
            while (opModeInInit()) idle();
            waitForStart();
            return;
        }

        Pose2d start = poseFieldToRR(startField); // convert to RR

        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        // Light gains (tune in Dashboard if you want)
        MecanumDrive.PARAMS.axialGain   = 3.0;
        MecanumDrive.PARAMS.lateralGain = 3.5;
        MecanumDrive.PARAMS.headingGain = 10.0;

        // Seed pose in INIT
        drive.localizer.setPose(start);

        // Build trajectory
        TrajectoryActionBuilder b = drive.actionBuilder(start);
        Pose2d cursor = start;

        for (Seg sF : segsField) {
            Vector2d tRR = vecFieldToRR(sF.x, sF.y);
            double tx = tRR.x, ty = tRR.y;
            double headRR = angFieldToRR(sF.headRad);     // only used on *Linear/Spline heading* calls
            double tanRR  = angFieldToRR(sF.tanRad);      // tangent from export (for splineTo*)

            switch (sF.type) {
                case SPLINE_TO:
                    b = b.splineTo(new Vector2d(tx, ty), tanRR);
                    // heading follows spline tangent; approximate cursor heading with tan
                    cursor = new Pose2d(tx, ty, tanRR);
                    break;

                case SPLINE_TO_CONSTANT_HEADING:
                    b = b.splineToConstantHeading(new Vector2d(tx, ty), tanRR);
                    // keep current heading
                    cursor = new Pose2d(tx, ty, cursor.heading.toDouble());
                    break;

                case SPLINE_TO_LINEAR_HEADING:
                    b = b.splineToLinearHeading(new Pose2d(tx, ty, headRR), tanRR);
                    cursor = new Pose2d(tx, ty, headRR);
                    break;

                case SPLINE_TO_SPLINE_HEADING:
                    b = b.splineToSplineHeading(new Pose2d(tx, ty, headRR), tanRR);
                    cursor = new Pose2d(tx, ty, headRR);
                    break;

                case LINE_TO_CONSTANT_HEADING: {
                    // Straight segment; use direction-to-target as tangent
                    double dir = Math.atan2(ty - cursor.position.y, tx - cursor.position.x);
                    b = b.splineToConstantHeading(new Vector2d(tx, ty), dir);
                    cursor = new Pose2d(tx, ty, cursor.heading.toDouble());
                } break;

                case LINE_TO_LINEAR_HEADING: {
                    double dir = Math.atan2(ty - cursor.position.y, tx - cursor.position.x);
                    b = b.splineToLinearHeading(new Pose2d(tx, ty, headRR), dir);
                    cursor = new Pose2d(tx, ty, headRR);
                } break;

                case LINE_TO: {
                    double dir = Math.atan2(ty - cursor.position.y, tx - cursor.position.x);
                    b = b.splineTo(new Vector2d(tx, ty), dir);
                    cursor = new Pose2d(tx, ty, dir);
                } break;

                case TURN:
                    b = b.turn(sF.turnRad);
                    cursor = new Pose2d(cursor.position.x, cursor.position.y,
                            cursor.heading.toDouble() + sF.turnRad);
                    break;
            }
        }

        Action action = b.build();

        // INIT telemetry + X to snap
        while (opModeInInit()) {
            if (gamepad1.x) drive.localizer.setPose(start);
            Pose2d pp = drive.localizer.getPose();
            telemetry.addLine("RRPathGen Runner (Center origin)");
            telemetry.addData("Start (FIELD)", "x=%.2f y=%.2f h=%.1f°",
                    startField.position.x, startField.position.y, Math.toDegrees(startField.heading.toDouble()));
            telemetry.addData("Start  (RR)  ", "x=%.2f y=%.2f h=%.1f°",
                    start.position.x, start.position.y, Math.toDegrees(start.heading.toDouble()));
            telemetry.addData("Pinpoint NOW ", "x=%.2f y=%.2f h=%.1f°",
                    pp.position.x, pp.position.y, Math.toDegrees(pp.heading.toDouble()));
            telemetry.addLine("Place robot at Start(RR). Press X to re-seed.");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Re-seed once after start
        drive.localizer.setPose(start);

        Actions.runBlocking(action);
    }

    // -------------------- Robust RRPathGen parser (center-origin) --------------------
    // Start: ...trajectorySequenceBuilder(new Pose2d(x, y, Math.toRadians(h)))...
    private static final Pattern START_POSE = Pattern.compile(
            "trajectory\\w*\\s*=\\s*drive\\.trajectorySequenceBuilder\\(\\s*new\\s+Pose2d\\(\\s*([-0-9.]+)\\s*,\\s*([-0-9.]+)\\s*,\\s*Math\\.toRadians\\(\\s*([-0-9.]+)\\s*\\)\\s*\\)\\s*\\)",
            Pattern.CASE_INSENSITIVE);

    private static final Pattern P_POSE = Pattern.compile(
            "new\\s+Pose2d\\(\\s*([-0-9.]+)\\s*,\\s*([-0-9.]+)\\s*,\\s*Math\\.toRadians\\(\\s*([-0-9.]+)\\s*\\)\\s*\\)",
            Pattern.CASE_INSENSITIVE);

    private static final Pattern P_VEC = Pattern.compile(
            "new\\s+Vector2d\\(\\s*([-0-9.]+)\\s*,\\s*([-0-9.]+)\\s*\\)",
            Pattern.CASE_INSENSITIVE);

    private static final Pattern P_RAD = Pattern.compile(
            "Math\\.toRadians\\(\\s*([-0-9.]+)\\s*\\)",
            Pattern.CASE_INSENSITIVE);

    private static Pose2d parseStartPose(String s) {
        Matcher m = START_POSE.matcher(s);
        if (!m.find()) return null;
        double x = Double.parseDouble(m.group(1));
        double y = Double.parseDouble(m.group(2));
        double hDeg = Double.parseDouble(m.group(3));
        return new Pose2d(x, y, Math.toRadians(hDeg)); // FIELD frame
    }

    private static List<Seg> parseSegmentsPreserveOrder(String s) {
        List<Seg> out = new ArrayList<>();
        String[] lines = s.split("\\r?\\n");
        for (String raw : lines) {
            String line = raw.trim();
            if (line.isEmpty()) continue;
            String lo = line.toLowerCase(Locale.ROOT);

            // ---- spline family ----
            if (lo.startsWith(".splinetosplineheading(")) {
                double[] p = readPose(line);
                double tan = readRad(line, 2);
                out.add(Seg.splineToSpline(p[0], p[1], p[2], tan));
            } else if (lo.startsWith(".splinetolinearheading(")) {
                double[] p = readPose(line);
                double tan = readRad(line, 2);
                out.add(Seg.splineToLinear(p[0], p[1], p[2], tan));
            } else if (lo.startsWith(".splinetoconstantheading(")) {
                double[] v = readVec(line);
                double tan = readRad(line, 1);
                out.add(Seg.splineToConst(v[0], v[1], tan));
            } else if (lo.startsWith(".splineto(")) {
                double[] v = readVec(line);
                double tan = readRad(line, 1);
                out.add(Seg.splineTo(v[0], v[1], tan));
            }

            // ---- line family ----
            else if (lo.startsWith(".linetolinearheading(")) {
                double[] p = readPose(line);
                out.add(Seg.lineToLinear(p[0], p[1], p[2]));
            } else if (lo.startsWith(".linetoconstantheading(")) {
                double[] pv = readPoseOrVec(line);
                out.add(Seg.lineToConst(pv[0], pv[1]));
            } else if (lo.startsWith(".lineto(")) {
                double[] pv = readPoseOrVec(line);
                out.add(Seg.lineTo(pv[0], pv[1]));
            }

            // ---- turn ----
            else if (lo.startsWith(".turn(")) {
                double ang = readRad(line, 1);
                out.add(Seg.turn(ang));
            }
        }
        return out;
    }

    // helpers: read Pose2d/Vector2d/radians
    private static double[] readPoseOrVec(String line) {
        Matcher mp = P_POSE.matcher(line);
        if (mp.find()) {
            return new double[]{ Double.parseDouble(mp.group(1)),
                    Double.parseDouble(mp.group(2)),
                    Math.toRadians(Double.parseDouble(mp.group(3))) };
        }
        Matcher mv = P_VEC.matcher(line);
        if (mv.find()) {
            return new double[]{ Double.parseDouble(mv.group(1)),
                    Double.parseDouble(mv.group(2)) };
        }
        throw new IllegalArgumentException("Pose2d/Vector2d not found: " + line);
    }
    private static double[] readPose(String line) {
        Matcher m = P_POSE.matcher(line);
        if (!m.find()) throw new IllegalArgumentException("Pose2d not found: " + line);
        return new double[]{ Double.parseDouble(m.group(1)),
                Double.parseDouble(m.group(2)),
                Math.toRadians(Double.parseDouble(m.group(3))) };
    }
    private static double[] readVec(String line) {
        Matcher m = P_VEC.matcher(line);
        if (!m.find()) throw new IllegalArgumentException("Vector2d not found: " + line);
        return new double[]{ Double.parseDouble(m.group(1)),
                Double.parseDouble(m.group(2)) };
    }
    private static double readRad(String line, int nth) {
        Matcher m = P_RAD.matcher(line);
        int i = 0;
        while (m.find()) {
            i++;
            if (i == nth) return Math.toRadians(Double.parseDouble(m.group(1)));
        }
        throw new IllegalArgumentException("Radians arg #" + nth + " not found: " + line);
    }

    // Segment model (FIELD frame values straight from export)
    private static class Seg {
        enum Type {
            SPLINE_TO, SPLINE_TO_CONSTANT_HEADING, SPLINE_TO_LINEAR_HEADING, SPLINE_TO_SPLINE_HEADING,
            LINE_TO, LINE_TO_CONSTANT_HEADING, LINE_TO_LINEAR_HEADING,
            TURN
        }
        final Type type;
        final double x, y, headRad, tanRad, turnRad;
        private Seg(Type t, double x, double y, double headRad, double tanRad, double turnRad) {
            this.type=t; this.x=x; this.y=y; this.headRad=headRad; this.tanRad=tanRad; this.turnRad=turnRad;
        }
        static Seg splineTo      (double x,double y,double tan){ return new Seg(Type.SPLINE_TO, x,y, 0,   tan, 0); }
        static Seg splineToConst (double x,double y,double tan){ return new Seg(Type.SPLINE_TO_CONSTANT_HEADING, x,y, 0,  tan, 0); }
        static Seg splineToLinear(double x,double y,double head,double tan){ return new Seg(Type.SPLINE_TO_LINEAR_HEADING, x,y, head, tan, 0); }
        static Seg splineToSpline(double x,double y,double head,double tan){ return new Seg(Type.SPLINE_TO_SPLINE_HEADING, x,y, head, tan, 0); }

        static Seg lineTo        (double x,double y){ return new Seg(Type.LINE_TO, x,y, 0,   0, 0); }
        static Seg lineToConst   (double x,double y){ return new Seg(Type.LINE_TO_CONSTANT_HEADING, x,y, 0, 0, 0); }
        static Seg lineToLinear  (double x,double y,double head){ return new Seg(Type.LINE_TO_LINEAR_HEADING, x,y, head, 0, 0); }

        static Seg turn(double ang){ return new Seg(Type.TURN, Double.NaN, Double.NaN, 0, 0, ang); }
    }
}
