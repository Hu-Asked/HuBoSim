import Structs.Line2D;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;

public class Lidar {
    enum Direction {
        FRONT,
        LEFT,
        RIGHT,
        BACK
    }
    Chassis chassis;
    double maxRange;
    Line2D[] sensors = new Line2D[4];
    double[] distFromWall = new double[4]; // front, left, right, back
    public Lidar(double maxRange, Chassis chassis) {
        this.maxRange = maxRange;
        this.chassis = chassis;
        updateSensorLines();
    }

    public void drawSensorLines(Graphics g) {
        g.setColor(Color.RED);
        for(Line2D sensor : sensors) {
            g.drawLine((int)sensor.start.x, (int)sensor.start.y, (int)sensor.end.x, (int)sensor.end.y);
        }
    }

    public void updateSensorLines() {
        Structs.Pose pose = SimMath.cartesianToPixels(this.chassis.pose);
        double x = pose.x;
        double y = pose.y;
        double angle = pose.heading;
        double length = SimMath.inchesToPixels(maxRange);
        sensors[Direction.FRONT.ordinal()] =  new Line2D(new Structs.Point(x, y), new Structs.Point(x + length * Math.cos(angle - Math.PI/2), y + length * Math.sin(angle - Math.PI/2)));
        sensors[Direction.LEFT.ordinal()] = new Line2D(new Structs.Point(x, y), new Structs.Point(x + length * Math.cos(angle + Math.PI), y + length * Math.sin(angle + Math.PI)));
        sensors[Direction.RIGHT.ordinal()] = new Line2D(new Structs.Point(x, y), new Structs.Point(x + length * Math.cos(angle), y + length * Math.sin(angle)));
        sensors[Direction.BACK.ordinal()] = new Line2D(new Structs.Point(x, y), new Structs.Point(x + length * Math.cos(angle + Math.PI/2), y + length * Math.sin(angle + Math.PI/2)));
        this.checkForIntersections(Main.field.walls);
    }

    public void checkForIntersections(Line2D[] walls) {
        boolean[] foundIntersection = new boolean[sensors.length];
        for(Direction dir : Direction.values()) {
            for(Structs.Line2D wall : walls) {
                Line2D sensor = sensors[dir.ordinal()];
                Structs.Point intersection = SimMath.getLineIntersection(sensor, wall);
                if(intersection != null) {
                    double dx = intersection.x - sensor.start.x;
                    double dy = intersection.y - sensor.start.y;
                    double distance = Math.hypot(dx, dy);
                    if(distance < SimMath.inchesToPixels(maxRange)) {
                        distFromWall[dir.ordinal()] = SimMath.pixelsToInches(distance);
                        foundIntersection[dir.ordinal()] = true;
                    }
                }
            }
            if(!foundIntersection[dir.ordinal()]) {
                distFromWall[dir.ordinal()] = -1;
            }
        }
    }
}
