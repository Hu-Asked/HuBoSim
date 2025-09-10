import Structs.Line2D;
import Structs.Point;
import Structs.Pose;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class PurePursuit {
    public Pose currentPose;
    public Chassis chassis;
    public double kPt;
    public double kPd;
    public double trackWidth;
    public int pathSegIndex = 0;
    public double pathSegmentScalarProgression = 0;
    public double pathFollowPercentage = 0;
    public double lookaheadDistance = 10;

    public double rHeading = 0;
    public double aHeading = 0;

    public boolean exit = false;

    public Structs.Point targetPoint = new Structs.Point(0,0);
    public ArrayList<Map.Entry<Structs.Point, Double>> actualPath = new ArrayList<>();

    public PurePursuit(Chassis chassis, double kPt, double kPd, double trackWidth) {
        this.chassis = chassis;
        this.kPt = kPt;
        this.kPd = kPd;
        this.trackWidth = trackWidth;
    }

    public ArrayList<Map.Entry<Line2D, Double>> processPath(final ArrayList<Map.Entry<Point, Double>> unprocessedPath) {
        ArrayList<Map.Entry<Line2D, Double>> path = new ArrayList<>();
        for (int i = 0; i < unprocessedPath.size() - 1; i++) {
            Line2D line = new Line2D(unprocessedPath.get(i).getKey(), unprocessedPath.get(i + 1).getKey());
            path.add(new HashMap.SimpleEntry<>(line, unprocessedPath.get(i).getValue()));
        }
        return path;
    }

    public void goToPosition(final Point target, double movePow) {
        double dx = target.x - currentPose.x;
        double dy = target.y - currentPose.y;
        double distance = Math.hypot(dx, dy);

        double absoluteHeading = Math.atan2(-dy, dx) + Math.PI / 2;
        double relativeHeading = MathPP.angleWrap(absoluteHeading - MathPP.angleWrap(currentPose.heading, true), true);
        this.rHeading = relativeHeading * 180/Math.PI;
        this.aHeading = absoluteHeading * 180/Math.PI;
        if(Math.abs(relativeHeading) > Math.PI / 3) {
            double turnPow = Math.clamp(kPt*relativeHeading, -2.5, 2.5) * movePow;
            chassis.leftDrive(-turnPow);
            chassis.rightDrive(turnPow);
            return;
        }

        double curve = MathPP.findCurvature(relativeHeading, distance);
        double turnPow = ((curve * trackWidth)/2.0) * movePow;

        chassis.leftDrive(movePow - turnPow);
        chassis.rightDrive(movePow + turnPow);
    }

    public Point getTargetPoint(final ArrayList<Line2D> path) {
        Map.Entry<Point, Double> bestIntersection = null;
        bestIntersection = new HashMap.SimpleEntry<>(new Point(0,0), -1.0);
        int segment = -1;
        boolean foundIntersection = false;
        for(int i = pathSegIndex; i < path.size(); i++) {
            ArrayList<Map.Entry<Point, Double>> intersections = MathPP.getCircleLineIntersection(currentPose, path.get(i), lookaheadDistance, true);
            if(!intersections.isEmpty()) {
                for(Map.Entry<Point, Double> intersection : intersections) {
                    if(intersection.getValue() > bestIntersection.getValue() || i > segment) {
                        bestIntersection = intersection;
                        segment = i;
                        foundIntersection = true;
                    }
                }
            }
        }
        if(foundIntersection && segment >= pathSegIndex) {
            pathSegIndex = segment;
            return this.targetPoint = bestIntersection.getKey();
        }
        return this.targetPoint = path.get(pathSegIndex).end;
    }

    public void initializePath(ArrayList<Line2D> path) {

        this.pathFollowPercentage = 0;
        this.pathSegmentScalarProgression = 0;
        this.pathSegIndex = 0;
        this.exit = false;

        this.actualPath = new ArrayList<>();

        double minDist = 1e3;
        for(int i = 0; i < path.size(); i++) {
            Line2D line = path.get(i);
            double dist = Math.hypot(line.start.x - currentPose.x, line.start.y - currentPose.y);
            if(dist < minDist) {
                minDist = dist;
                pathSegIndex = i;
            }
        }
    }
    public void updateSegmentProgression(final ArrayList<Line2D> path) {
        if(path.isEmpty() || pathSegIndex >= path.size()) return;
        Line2D currentSegment = path.get(pathSegIndex);
        double projectionScalar = MathPP.findProjectionScalar(currentSegment, new Point(currentPose.x, currentPose.y));
        projectionScalar = Math.max(0.0, Math.min(1.0, projectionScalar));
        pathSegmentScalarProgression = projectionScalar;

        if(path.size() > 1) {
            double segmentProgress = pathSegIndex / (double)(path.size()-1);
            double intraSegmentProgress = pathSegmentScalarProgression / (double)(path.size()-1);
            pathFollowPercentage = (segmentProgress + intraSegmentProgress)*100;
        }
    }
    public boolean isPathComplete(final ArrayList<Line2D> path) {
        if(pathFollowPercentage >= 90 || pathSegIndex >= path.size() - 1) {
            Line2D lastSegment = path.get(path.size() - 1);
            Point pathEnd = lastSegment.end;
            double distToEnd = Math.hypot(pathEnd.x - currentPose.x, pathEnd.y - currentPose.y);
            return distToEnd < lookaheadDistance;
        }
        return false;
    }

    public ArrayList<Line2D> getStrippedPath(final ArrayList<Map.Entry<Point, Double>> unprocessedPath) {
        ArrayList<Line2D> strippedPath = new ArrayList<>();
        ArrayList<Map.Entry<Line2D, Double>> path = processPath(unprocessedPath);
        for(Map.Entry<Line2D, Double> entry : path) {
            strippedPath.add(entry.getKey());
        }
        return strippedPath;
    }
    public void followPath(final ArrayList<Map.Entry<Point, Double>> unprocessedPath, double lookaheadDistance, double maxSpeed, double distBetweenSegments) {
        this.lookaheadDistance = lookaheadDistance;
        ArrayList<Line2D> strippedPath  = getStrippedPath(unprocessedPath);

        Point target = getTargetPoint(strippedPath);
        if(isPathComplete(strippedPath)) {
            chassis.leftDrive(0);
            chassis.rightDrive(0);
            exit = true;
            return;
        }
        actualPath.add(new AbstractMap.SimpleEntry<>(new Point(currentPose.x, currentPose.y), 0.0));
        double speedMultiplier = unprocessedPath.get(Math.max(0, (int) (pathSegIndex - (lookaheadDistance / distBetweenSegments)))).getValue();
        goToPosition(target, maxSpeed * speedMultiplier/100);
        updateSegmentProgression(strippedPath);

    }
}
