package util;

public class VelocityVector {
    public double magnitude;
    public double directionRads;

    public VelocityVector(double magnitude, double directionRads) {
        this.magnitude = magnitude;
        this.directionRads = directionRads;
    }

    public double getXComponent() {
        return magnitude * Math.cos(directionRads);
    }
    public double getYComponent() {
        return magnitude * Math.sin(directionRads);
    }
}
