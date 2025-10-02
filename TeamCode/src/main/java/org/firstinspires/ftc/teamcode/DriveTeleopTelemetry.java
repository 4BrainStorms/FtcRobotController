package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayDeque;
import java.util.Deque;

public class DriveTeleopTelemetry {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Deque<Pose2d> trail = new ArrayDeque<>();
    private final int maxTrail;
    private double arrowScale = 6.0; // inches per full-stick for the command arrow

    public DriveTeleopTelemetry() { this(200); }
    public DriveTeleopTelemetry(int maxTrailPoints) {
        this.maxTrail = Math.max(10, maxTrailPoints);
    }

    public void setArrowScale(double inchesPerFullStick) {
        this.arrowScale = inchesPerFullStick;
    }

    public void resetTrail() {
        trail.clear();
    }

    /**
     * @param drive           your MecanumDrive
     * @param robotCmd        robot-frame command (forward, left) in [-1..1]
     * @param omegaCmd        rotational command (CCW +) in [-1..1]
     * @param fieldCentric    true if field-centric mode is active
     * @param slowScalar0to1  0..1 scalar applied for slow mode (e.g., 0.3-1.0)
     */
    public void send(MecanumDrive drive,
                     Vector2d robotCmd,
                     double omegaCmd,
                     boolean fieldCentric,
                     double slowScalar0to1) {

        Pose2d p = drive.localizer.getPose();

        if (trail.size() >= maxTrail) trail.removeFirst();
        trail.addLast(p);

        TelemetryPacket packet = new TelemetryPacket();

        // Scalars for DS dashboard too
        packet.put("mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
        packet.put("x", p.position.x);
        packet.put("y", p.position.y);
        packet.put("headingDeg", Math.toDegrees(p.heading.toDouble()));
        packet.put("cmdForward", robotCmd.x);
        packet.put("cmdLeft", robotCmd.y);
        packet.put("cmdOmega", omegaCmd);
        packet.put("slowPct", slowScalar0to1 * 100.0);

        Canvas c = packet.fieldOverlay();

        // 1) Trail polyline
        double[] xs = new double[trail.size()];
        double[] ys = new double[trail.size()];
        int i = 0;
        for (Pose2d t : trail) {
            xs[i] = t.position.x;
            ys[i] = t.position.y;
            i++;
        }
        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xs, ys);

        // 2) Robot icon (use Quickstart Drawing if present; else fallback)
        try {
            Drawing.drawRobot(c, p);
        } catch (Throwable ignored) {
            drawSimpleRobot(c, p);
        }

        // 3) Command arrow (convert robot-frame command to field-frame for overlay)
        double h = p.heading.toDouble();
        double cf = Math.cos(h), sf = Math.sin(h);
        double worldX = robotCmd.x * cf - robotCmd.y * sf;
        double worldY = robotCmd.x * sf + robotCmd.y * cf;

        double sx = p.position.x, sy = p.position.y;
        double ex = sx + worldX * arrowScale;
        double ey = sy + worldY * arrowScale;
        c.setStroke("#4CAF50FF");
        c.setStrokeWidth(2);
        c.strokeLine(sx, sy, ex, ey);

        dashboard.sendTelemetryPacket(packet);
    }

    // Fallback robot glyph if Drawing.java isn't present
    private static void drawSimpleRobot(Canvas c, Pose2d p) {
        double r = 4.5; // ~9" robot half-size
        double h = p.heading.toDouble();
        double cx = p.position.x, cy = p.position.y;
        double x1 = cx + r * Math.cos(h);
        double y1 = cy + r * Math.sin(h);
        double x2 = cx + r * Math.cos(h + 2.5);
        double y2 = cy + r * Math.sin(h + 2.5);
        double x3 = cx + r * Math.cos(h - 2.5);
        double y3 = cy + r * Math.sin(h - 2.5);
        c.setFill("#3F51B57A");
        c.fillPolygon(new double[]{x1, x2, x3}, new double[]{y1, y2, y3});
    }
}
