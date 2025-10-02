package huasked.hubosim;

import huasked.hubosim.util.Line;
import huasked.hubosim.util.Point;
import huasked.hubosim.util.Pose;

import java.awt.*;
import java.util.Arrays;

public class Lidar {
    enum Direction {
        FRONT,
        RIGHT,
        BACK,
        LEFT
    }

    Chassis chassis;
    double maxRange;
    Line[] sensors = new Line[4];
    double[] distFromWall = new double[4]; // front, right, back, left
    int[] detectedWall = {-1, -1, -1, -1};

    public Lidar(double maxRange, Chassis chassis) {
        this.maxRange = maxRange;
        this.chassis = chassis;
        updateSensorLines();
    }

    public void drawSensorLines(Graphics g) {
        g.setColor(Color.RED);
        for (Line sensor : sensors) {
            g.drawLine((int) sensor.start.x, (int) sensor.start.y, (int) sensor.end.x, (int) sensor.end.y);
        }
    }

    public void updateSensorLines() {
        Pose pose = SimMath.cartesianToPixels(this.chassis.pose);
        double x = pose.x;
        double y = pose.y;
        double angle = pose.heading;
        double length = SimMath.inchesToPixels(maxRange);
        sensors[Direction.FRONT.ordinal()] = new Line(new Point(x, y), new Point(x + length * Math.cos(angle - Math.PI / 2), y + length * Math.sin(angle - Math.PI / 2)));
        sensors[Direction.RIGHT.ordinal()] = new Line(new Point(x, y), new Point(x + length * Math.cos(angle), y + length * Math.sin(angle)));
        sensors[Direction.BACK.ordinal()] = new Line(new Point(x, y), new Point(x + length * Math.cos(angle + Math.PI / 2), y + length * Math.sin(angle + Math.PI / 2)));
        sensors[Direction.LEFT.ordinal()] = new Line(new Point(x, y), new Point(x + length * Math.cos(angle + Math.PI), y + length * Math.sin(angle + Math.PI)));
        this.checkForIntersections(Main.field.walls);
    }

    public void checkForIntersections(Line[] walls) {
        boolean[] foundIntersection = new boolean[sensors.length];
        for (Direction dir : Direction.values()) {
            for (Line wall : walls) {
                Line sensor = sensors[dir.ordinal()];
                Point intersection = SimMath.getLineIntersection(sensor, wall);
                if (intersection != null) {
                    double dx = intersection.x - sensor.start.x;
                    double dy = intersection.y - sensor.start.y;
                    double distance = Math.hypot(dx, dy);
                    if (distance < SimMath.inchesToPixels(maxRange)) {
                        distFromWall[dir.ordinal()] = SimMath.pixelsToInches(distance) + SimMath.getGaussianError(1.0);
                        detectedWall[dir.ordinal()] = Arrays.asList(walls).indexOf(wall);
                        foundIntersection[dir.ordinal()] = true;
                    }
                }
            }
            if (!foundIntersection[dir.ordinal()]) {
                distFromWall[dir.ordinal()] = -1;
                detectedWall[dir.ordinal()] = -1;
            }
        }
    }
}
