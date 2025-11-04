package huasked.hubosim;

import huasked.hubosim.util.Line;
import huasked.hubosim.util.Point;
import huasked.hubosim.util.Pose;

import java.util.Random;

import javax.swing.JPanel;

public class Base extends JPanel {
    public static Random rd = new Random();

    protected static final double inchesToPixelsRatio = 140.0/880.0;

    public static Point getLineIntersection(Line line1, Line line2) {
        double x1 = line1.start.x, y1 = line1.start.y;
        double x2 = line1.end.x, y2 = line1.end.y;
        double x3 = line2.start.x, y3 = line2.start.y;
        double x4 = line2.end.x, y4 = line2.end.y;

        double dn = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (Math.abs(dn) < 1e-8) {
            return null; // Lines are parallel
        }

        double px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / dn;
        double py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / dn;

        // Check if intersection is within both segments
        if (px < Math.min(x1, x2) - 1e-8 || px > Math.max(x1, x2) + 1e-8 ||
            px < Math.min(x3, x4) - 1e-8 || px > Math.max(x3, x4) + 1e-8 ||
            py < Math.min(y1, y2) - 1e-8 || py > Math.max(y1, y2) + 1e-8 ||
            py < Math.min(y3, y4) - 1e-8 || py > Math.max(y3, y4) + 1e-8) {
            return null;
        }

        return new Point(px, py);
    }

    public static double getGaussianError(double errorMarginPct) {
        return rd.nextGaussian() * (errorMarginPct / 100.0);
    }

    protected static double inchesToPixels(double inches) { return inches / inchesToPixelsRatio; }

    protected static double pixelsToInches(double pixels) { return pixels * inchesToPixelsRatio; }

    protected static Pose pixelsToCartesian(Pose pose) { return new Pose(pixelsToInches(pose.x) - 70, -pixelsToInches(pose.y) + 70, pose.heading); }

    protected static Pose cartesianToPixels(Pose pose) { return new Pose(inchesToPixels(pose.x + 70 + 3), -inchesToPixels(pose.y - 70 + 3), pose.heading); }

}
