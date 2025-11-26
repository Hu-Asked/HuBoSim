package huasked.hubosim.algorithm;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.ArrayList;

import huasked.hubosim.BaseRenderObject;
import huasked.hubosim.chassis.TankDrive;
import huasked.hubosim.util.Pose;

public class MCL extends BaseRenderObject {
    private final ArrayList<Pose> particles;
    private final ArrayList<Double> weights;
    private final Lidar lidar;
    private final TankDrive chassis;
    public double updateX;
    public double updateY;
    public double updateTheta;
    private int numParticles;
    private double minDistTravel = 1;
    private double distSinceLastUpdate = 0;
    private double totalWeight = 0;
    private double expectedReadingOffsetLat = 0;
    private double expectedReadingOffsetSides = 0;
    private Pose prevPose;
    private Pose currPose;
    public Pose estimatedPose;
    private double FIELD_SIZE = 140;
    private double MARGIN = 3;

    public MCL(int numParticles, double minDistTravel, double expectedReadingOffsetLat, double expecctedReadingOffsetSides, Lidar lidar, TankDrive chassis) {
        this.numParticles = numParticles;
        this.minDistTravel = minDistTravel;
        this.lidar = lidar;
        this.chassis = chassis;
        this.particles = new ArrayList<>();
        this.weights = new ArrayList<>();
        this.prevPose = chassis.pose;
        this.currPose = prevPose;
        this.expectedReadingOffsetLat = expectedReadingOffsetLat;
        this.expectedReadingOffsetSides = expecctedReadingOffsetSides;
        this.estimatedPose = new Pose(chassis.pose.x, chassis.pose.y, chassis.pose.heading);
        this.FIELD_SIZE = inchesToPixels(FIELD_SIZE);
        this.MARGIN = inchesToPixels(MARGIN);
        initializeParticles();
    }

    @Override
    public void update() {
        this.prevPose = this.currPose;
        this.currPose = new Pose(this.chassis.pose.x, this.chassis.pose.y, this.chassis.pose.heading);
        this.updateX = this.currPose.x - this.prevPose.x;
        this.updateY = this.currPose.y - this.prevPose.y;
        this.updateTheta = this.currPose.heading - this.prevPose.heading;
        updateParticles();
    }

    @Override
    public void render(Graphics2D g2d) {
        drawParticles(g2d, 4);
    }

    @Override
    public void debugInfo(Graphics g, int x, int y) {
        g.drawString(String.format("Estimated Pose : %.2f | %.2f | %.2f", estimatedPose.x, estimatedPose.y, estimatedPose.heading), x, y);
    }

    private Pose redistributeParticle() {
        double x = Math.random() * (FIELD_SIZE - MARGIN) + MARGIN;
        double y = Math.random() * (FIELD_SIZE - MARGIN) + MARGIN;
        return pixelsToCartesian(new Pose(x, y, Math.random() * 2 * Math.PI));
    }

    private void initializeParticles() {
        for (int i = 0; i < numParticles; i++) {
            particles.add(redistributeParticle());
            weights.add(1.0);
            totalWeight += 1.0;
        }
    }

    public void drawParticles(Graphics2D g, int radius) {
        for (Pose p : particles) {
            Pose pixelPose = cartesianToPixels(p);
            g.setColor(Color.BLUE);
            int px = (int) pixelPose.x;
            int py = (int) pixelPose.y;
            int newRadius = radius;
            //          int newRadius = (int) (radius * (weighParticle(p) / (totalWeight / numParticles)));
            //          if (newRadius < 2) newRadius = 2;
            g.fillOval(px - newRadius / 2, py - newRadius / 2, newRadius, newRadius);
            //draw line representing heading
            g.setColor(Color.BLACK);
            int hx = (int) (px + newRadius * Math.cos(p.heading - Math.PI / 2));
            int hy = (int) (py + newRadius * Math.sin(p.heading - Math.PI / 2));
            g.drawLine(px, py, hx, hy);
        }
    }


    private void updateParticles() {
        double distance = Math.hypot(updateX, updateY);
        distSinceLastUpdate += distance;
        boolean isReversing = chassis.isReversing();
        if (isReversing) {
            distance *= -1;
        }

        this.totalWeight = 0;
        double avgX = 0;
        double avgY = 0;
        double avgTheta = 0;
        for (int i = 0; i < numParticles; i++) {
            Pose particle = particles.get(i);
            double deltaTh = updateTheta;
            particle.heading += deltaTh;

            double noisyDistance = distance + getGaussianError(2.0);
            particle.x += noisyDistance * -Math.cos(particle.heading + Math.PI / 2);
            particle.y += noisyDistance * Math.sin(particle.heading + Math.PI / 2);
            avgX += particle.x;
            avgY += particle.y;
            avgTheta += particle.heading;
            if (isOutOfBounds(particle)) {
                Pose newPose = redistributeParticle();
                particle.x = newPose.x;
                particle.y = newPose.y;
                particle.heading = newPose.heading;
            }
        }

        this.estimatedPose = new Pose(avgX, avgY, avgTheta);
    }

    private double calculateEffectiveSampleSize() {
        return 0;
    }

    private double weighParticle(Pose p) {
        return 0;
    }

    private void normalizeWeights() {

    }

    private boolean isOutOfBounds(Pose p) {
        return !(p.x > -67 && p.x < 67 && p.y > -67 && p.y < 67);
    }

    public double getParticleReading(Pose p, Lidar.Direction sensorDir) {
        double sensorHeading = MathPP.angleWrap(p.heading + sensorDir.ordinal() * Math.PI / 2, true);
        int wall = lidar.detectedWall[sensorDir.ordinal()];
        if (wall == -1) {
            return -1;
        }
        double relativeHeading = 0;
        double offsetFromWall = 0;
        switch (wall) {
            case 0 -> {
                offsetFromWall = 70 - p.y;
                relativeHeading = MathPP.angleWrap(sensorHeading, true);
            }
            case 1 -> {
                offsetFromWall = 70 - p.x;
                relativeHeading = MathPP.angleWrap(Math.PI / 2 - sensorHeading, true);
            }
            case 2 -> {
                offsetFromWall = 70 + p.y;
                relativeHeading = MathPP.angleWrap(Math.PI - sensorHeading, true);
            }
            case 3 -> {
                offsetFromWall = 70 + p.x;
                relativeHeading = MathPP.angleWrap(3 * Math.PI / 2 - sensorHeading, true);
            }
        }
        double res = offsetFromWall + relativeHeading;
        if (sensorDir == Lidar.Direction.FRONT || sensorDir == Lidar.Direction.BACK) {
            res += expectedReadingOffsetLat;
        }
        else {
            res += expectedReadingOffsetSides;
        }
        return res;
    }

    private void resampleParticles() {

    }
}
