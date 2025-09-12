package util;

public class Pose {
    public double x;
    public double y;
    public double heading; // in radians

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
