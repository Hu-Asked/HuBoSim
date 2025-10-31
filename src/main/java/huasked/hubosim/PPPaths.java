package huasked.hubosim;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.HashMap;

import huasked.hubosim.util.Point;

public class PPPaths extends Base {
    public static class PathPoint {
        public final Point point;
        public final double speed;
        public PathPoint(double x, double y, double speed) {
            this.point = new Point(x, y);
            this.speed = speed;
        }
    }

    public static void renderPath(Graphics g, ArrayList<PathPoint> path, Color c, double WIDTH, double HEIGHT) {
        if(!path.isEmpty()) {
            Graphics2D g2d = (Graphics2D) g;
            g2d.setColor(c);
            g2d.setStroke(new BasicStroke(2.0f));
            for (int i = 0; i < path.size() - 1; i++) {
                Point start = path.get(i).point;
                Point end = path.get(i + 1).point;
                Line2D line = new Line2D.Double(
                    inchesToPixels(start.x) + WIDTH / 2,
                    inchesToPixels(-start.y) + HEIGHT / 2,
                    inchesToPixels(end.x) + WIDTH / 2,
                    inchesToPixels(-end.y) + HEIGHT / 2
                );
                g2d.draw(line);
            }
        }
    }
    private static HashMap<String, ArrayList<PathPoint>> paths = new HashMap<>();
    public static ArrayList<PathPoint> activePath = new ArrayList<>();
    public static void addPath(String name, ArrayList<PathPoint> path) {
        paths.put(name, path);
    }
    public static void setActive(String name) {
        activePath = paths.get(name);
    }
    public static void init() {
        ArrayList<PathPoint> path = new ArrayList<>();
        path.add(point(-2,   15,   98.40));
        path.add(point(-3.657,   16.121,   98.40));
        path.add(point(-5.306,   17.252,   98.30));
        path.add(point(-6.945,   18.398,   98.30));
        path.add(point(-8.567,   19.568,   98.21));
        path.add(point(-10.179,   20.752,   98.13));
        path.add(point(-11.779,   21.952,   98.08));
        path.add(point(-13.361,   23.175,   98.08));
        path.add(point(-14.929,   24.417,   98.06));
        path.add(point(-16.482,   25.677,   98.0));
        path.add(point(-18.02,   26.956,   98.17));
        path.add(point(-19.542,   28.252,   98.3));
        path.add(point(-21.05,   29.566,   98.5));
        path.add(point(-22.545,   30.895,   98.94));
        path.add(point(-24.028,   32.237,   99.43));
        path.add(point(-25.504,   33.586,   99.92));
        path.add(point(-26.979,   34.937,   99.11));
        path.add(point(-28.461,   36.28,   96.98));
        path.add(point(-29.96,   37.604,   95.68));
        path.add(point(-31.488,   38.895,   94.14));
        path.add(point(-33.064,   40.125,   91.4));
        path.add(point(-34.694,   41.283,   90.14));
        path.add(point(-36.399,   42.329,   88.48));
        path.add(point(-38.167,   43.262,   87.47));
        path.add(point(-40.005,   44.05,   84.52));
        path.add(point(-41.889,   44.718,   81.46));
        path.add(point(-43.817,   45.248,   78.28));
        path.add(point(-45.771,   45.675,   74.96));
        path.add(point(-47.742,   46.012,   71.));
        path.add(point(-49.725,   46.27,   67.85));
        path.add(point(-51.716,   46.463,   64.00));
        path.add(point(-53.711,   46.6,   59.90));
        path.add(point(-55.709,   46.691,   55.50));
        path.add(point(-57.708,   46.746,   50.71));
        path.add(point(-59.708,   46.77,   45.43));
        path.add(point(-61.708,   46.77,   39.45));
        path.add(point(-63.707,   46.748,   32.37));
        path.add(point(-65.707,   46.7,   23.24));
        path.add(point(-67.833,   46.639, 0));
        path.add(point(-67.833,   46.639, 0));
        path.add(point(-87.825,   46.062, 0));

        paths.put("elims", path);

    }
    private static PathPoint point(double x, double y, double speed) {
        return new PathPoint(x, y, speed);
    }
}
