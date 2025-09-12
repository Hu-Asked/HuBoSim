import jdk.jshell.execution.Util;
import util.Pose;

import java.awt.*;
import java.util.ArrayList;

public class MCL {
    private ArrayList<util.Pose> particles;
    private ArrayList<Double> weights;
    private Lidar lidar;
    private Chassis chassis;
    public double updateX;
    public double updateY;
    public double updateTheta;
    private int numParticles;
    private Pose prevPose;
    private Pose currPose;

    public MCL(int numParticles, Lidar lidar, Chassis chassis) {
        this.numParticles = numParticles;
        this.lidar = lidar;
        this.chassis = chassis;
        this.particles = new ArrayList<>();
        this.prevPose = chassis.pose;
        this.currPose = prevPose;
        initializeParticles();
    }

    private util.Pose redistField() {
        double FIELD_SIZE = Main.FIELD_SIZE;
        double MARGIN = Main.field.MARGIN;
        double x = Math.random() * (FIELD_SIZE - MARGIN) + MARGIN;
        double y = Math.random() * (FIELD_SIZE - MARGIN) + MARGIN;
        return SimMath.pixelsToCartesian(new util.Pose(x, y, Math.random() * 2 * Math.PI));
    }
    private void initializeParticles() {
        for (int i = 0; i < numParticles; i++) {
            particles.add(redistField());
        }
    }

    public void drawParticles(Graphics g, int radius) {
        boolean ded = false;
        for (util.Pose p : particles) {
            util.Pose pixelPose = SimMath.cartesianToPixels(p);
            g.setColor(Color.BLUE);
            int px = (int) pixelPose.x;
            int py = (int) pixelPose.y;
            g.fillOval(px - radius / 2, py - radius / 2, radius, radius);
            //draw line representing heading
            g.setColor(Color.BLACK);
            int hx = (int) (px + radius * Math.cos(p.heading - Math.PI / 2));
            int hy = (int) (py + radius * Math.sin(p.heading - Math.PI / 2));
            g.drawLine(px, py, hx, hy);
        }
    }

    public void update() {
        this.prevPose = this.currPose;
        this.currPose = new util.Pose(this.chassis.pose.x, this.chassis.pose.y, this.chassis.pose.heading);
        this.updateX = this.currPose.x - this.prevPose.x;
        this.updateY = this.currPose.y - this.prevPose.y;
        this.updateTheta = this.currPose.heading - this.prevPose.heading;
        updateParticles();
    }

    private void updateParticles() {
        double moveDirection = Math.atan2(updateY, updateX);
        double distance = Math.hypot(updateX, updateY);
        boolean isReversing = (chassis.leftVelocity < 0 || chassis.rightVelocity < 0);
        if (isReversing) distance *= -1;
        for (util.Pose particle : particles) {
            double dth = updateTheta;
            particle.heading += dth;
            double noisyDistance = distance * SimMath.getGaussianError(1.0);
            particle.x += noisyDistance * -Math.cos(particle.heading + Math.PI/2);
            particle.y += noisyDistance * Math.sin(particle.heading + Math.PI/2);
            if (isOutOfBounds(particle)) {
                util.Pose newPose = redistField();
                particle.x = newPose.x;
                particle.y = newPose.y;
                particle.heading = newPose.heading;
            }
        }
    }

    private boolean isOutOfBounds(util.Pose p) {
        return !(p.x > -67 && p.x < 67 && p.y > -67 && p.y < 67);
    }

    private double getExpectedReading(util.Pose p, Lidar.Direction sensorDir) {
        int wall = lidar.detectedWall[sensorDir.ordinal()];
        if (wall == -1) return -1;
        double relativeHeading = 0;
        double offsetFromWall = 0;
        switch (wall) {
            case 1 -> {
                offsetFromWall = 70 - p.y;
                relativeHeading = p.heading;
            }
            case 2 -> {
                offsetFromWall = -70 + p.x;
                relativeHeading = p.heading - 3*Math.PI/2;
            }
            case 3 -> {
                offsetFromWall = 70 - p.x;
                relativeHeading = p.heading - Math.PI/2;
            }
            case 4 -> {
                offsetFromWall = -70 + p.y;
                relativeHeading = p.heading - Math.PI;
            }
        }
        return offsetFromWall / Math.cos(relativeHeading + Math.PI/2);
    }
    private boolean weighParticle(util.Pose particle) {
        double weight = 1.0;
        double[] lidarReadings = lidar.distFromWall;
        for(int i = 0; i < lidarReadings.length; i++) {
            double expected = getExpectedReading(particle, Lidar.Direction.values()[i]);
            double actual = lidarReadings[i];
            if(actual < 0 || expected < 0) continue;
            double diff = actual - expected;

        }
    }

    private void resampleParticles() {

    }

}
