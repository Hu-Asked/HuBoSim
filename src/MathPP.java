import util.Line2D;
import util.Point;
import util.Pose;

import java.util.ArrayList;
import java.util.Map;

public class MathPP {
    static double M_PI = Math.PI;
    public static double toRads(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double angleWrap(double angle, boolean radians) {
        if (radians) {
            while(angle> M_PI) angle -= 2*M_PI;
            while(angle< -M_PI) angle += 2*M_PI;
        } else {
            while(angle> 180) angle -= 360;
            while(angle< -180) angle += 360;
        }
        return angle;
    }

    public static double findCurvature(double turnError, double distance) {
        if(distance < 0.01) return 0;
        return (2*Math.sin(turnError) / distance);
    }

    public static double dotProduct(final Point vec1, final Point vec2) {
        return (vec1.x * vec2.x) + (vec1.y * vec2.y);
    }

    public static double findProjectionScalar(final Line2D pathSegment, final Point point) {
        Point pathVec = new Point(pathSegment.end.x - pathSegment.start.x, pathSegment.end.y - pathSegment.start.y);
        Point pointVec = new Point(point.x - pathSegment.start.x, point.y - pathSegment.start.y);

        double magnitude = Math.hypot(pathVec.x, pathVec.y);

        return dotProduct(pathVec, pointVec) / (magnitude * magnitude);
    }

    public static ArrayList<Map.Entry<Point, Double>> getCircleLineIntersection (final Pose robotPose, final Line2D line, double lookaheadDistance, boolean getBoundedIntersections) {
        ArrayList<Map.Entry<Point, Double>> intersections = new ArrayList<>();

        double x1 = line.start.x;
        double y1 = line.start.y;
        double x2 = line.end.x;
        double y2 = line.end.y;

        double dx = x2-x1;
        double dy = y2-y1;
        double a = robotPose.x;
        double b = robotPose.y;

        double A = dx*dx + dy*dy;
        double B = 2*(dx*(x1-a) + dy*(y1-b));
        double C = (x1-a)*(x1-a) + (y1-b)*(y1-b) - lookaheadDistance*lookaheadDistance;
        double delta = B*B - 4*A*C;

        if(Math.abs(A) < 1e-3) {
            double dist = Math.hypot(x1-a, y1-b);
            if(Math.abs(dist - lookaheadDistance) < 1e-2) {
                intersections.add(Map.entry(new Point(x1, y1), 0.0));
            }
            return intersections;
        }

        if(delta < 0) return intersections;

        double t1 = (-B + Math.sqrt(delta)) / (2*A);
        double t2 = (-B - Math.sqrt(delta)) / (2*A);
        double ix1 = x1 + t1*dx;
        double iy1 = y1 + t1*dy;
        double ix2 = x1 + t2*dx;
        double iy2 = y1 + t2*dy;
        if (getBoundedIntersections) {
            if (t1 >= 0 && t1 <= 1) {
                intersections.add(Map.entry(new Point(ix1, iy1), t1));
            }
            if (t2 >= 0 && t2 <= 1) {
                intersections.add(Map.entry(new Point(ix2, iy2), t2));
            }
        } else {
            intersections.add(Map.entry(new Point(ix1, iy1), t1));
            intersections.add(Map.entry(new Point(ix2, iy2), t2));
        }

        return intersections;

    }



}
