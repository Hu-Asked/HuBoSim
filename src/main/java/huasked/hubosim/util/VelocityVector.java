package huasked.hubosim.util;

public class VelocityVector {
    public double magnitude;
    private double directionRads;
    public VelocityVector(double magnitude, double directionRads) {
        this.magnitude = magnitude;
        setDirection(directionRads);
    }

    public void setDirection(double directionRads) {
        this.directionRads = directionRads - Math.PI / 2;
    }

    public double getDirection() {
        return directionRads;
    }

    public double getXComponent() {
        return magnitude * Math.cos(directionRads);
    }

    public double getYComponent() {
        return magnitude * Math.sin(directionRads);
    }
}
