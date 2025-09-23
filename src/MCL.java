import util.Pose;

import java.awt.*;
import java.util.ArrayList;

public class MCL {
    private final ArrayList<util.Pose> particles;
    private final ArrayList<Double> weights;
    private final Lidar lidar;
    private final Chassis chassis;
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
    public util.Pose estimatedPose;

    public MCL(int numParticles, double minDistTravel, double expectedReadingOffsetLat, double expecctedReadingOffsetSides, Lidar lidar, Chassis chassis) {
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
        this.estimatedPose = new util.Pose(chassis.pose.x, chassis.pose.y, chassis.pose.heading);
        initializeParticles();
    }

    private util.Pose redistributeParticle() {
        double FIELD_SIZE = Main.FIELD_SIZE;
        double MARGIN = Main.field.MARGIN;
        double x = Math.random() * (FIELD_SIZE - MARGIN) + MARGIN;
        double y = Math.random() * (FIELD_SIZE - MARGIN) + MARGIN;
        return SimMath.pixelsToCartesian(new util.Pose(x, y, Math.random() * 2 * Math.PI));
    }
    private void initializeParticles() {
        for (int i = 0; i < numParticles; i++) {
            particles.add(redistributeParticle());
            weights.add(1.0);
            totalWeight += 1.0;
        }
    }

    public void drawParticles(Graphics g, int radius) {
        for (util.Pose p : particles) {
            util.Pose pixelPose = SimMath.cartesianToPixels(p);
            g.setColor(Color.BLUE);
            int px = (int) pixelPose.x;
            int py = (int) pixelPose.y;
            //make radius proportional to weight
            int newRadius = radius;
//          int newRadius = (int) (radius * (weighParticle(p) / (totalWeight / numParticles)));
//          if (newRadius < 2) newRadius = 2;
            g.fillOval(px - newRadius / 2, py - newRadius / 2, newRadius, newRadius);
            //draw line representing heading
            g.setColor(Color.BLACK);
            int hx = (int) (px + newRadius * Math.cos(p.heading - Math.PI / 2));
            int hy = (int) (py + newRadius * Math.sin(p.heading - Math.PI / 2));
            g.drawLine(px, py, hx, hy);

            double weight = weights.get(particles.indexOf(p));
            g.drawString(String.format("%.10f", weight), px + newRadius / 2, py + newRadius / 2);
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
        double distance = Math.hypot(updateX, updateY);
        distSinceLastUpdate += distance;
        boolean isReversing = (chassis.leftVelocity < 0 || chassis.rightVelocity < 0);
        if (isReversing) distance *= -1;
        this.totalWeight = 0;
        double avgX = 0;
        double avgY = 0;
        double avgTheta = 0;
        for (int i = 0; i < numParticles; i++) {
            util.Pose particle = particles.get(i);
            double dth = updateTheta;
            particle.heading += dth;

            double noisyDistance = distance + SimMath.getGaussianError(2.0);
            particle.x += noisyDistance * -Math.cos(particle.heading + Math.PI/2);
            particle.y += noisyDistance * Math.sin(particle.heading + Math.PI/2);
            avgX += particle.x;
            avgY += particle.y;
            avgTheta += particle.heading;
            if (isOutOfBounds(particle)) {
                util.Pose newPose = redistributeParticle();
                particle.x = newPose.x;
                particle.y = newPose.y;
                particle.heading = newPose.heading;
            }
            double weight = weighParticle(particle);
            weights.set(i, weight);
            totalWeight += weight;
        }
        normalizeWeights();
        avgY = avgY / numParticles;
        avgX = avgX / numParticles;
        avgTheta = avgTheta / numParticles;
    }

    private double weighParticle(util.Pose p) {
        double weight = 1.0;
        for (Lidar.Direction dir : Lidar.Direction.values()) {
            double lidarReading = lidar.distFromWall[dir.ordinal()];
            double particleReading = getParticleReading(p, dir);
            double error = lidarReading - particleReading;
            error /= 1e5;
            double sigma = 1.0;
            double variance = sigma*sigma;
            double gaussian = 1/(sigma*Math.sqrt(2*Math.PI));
            gaussian *= Math.exp(-(error*error)/(2*variance));
            weight *= gaussian;
        }
        return weight;
    }

    private void normalizeWeights() {
        double sum = 0;
        for (double w : weights) {
            sum += w;
        }
        for (int i = 0; i < weights.size(); i++) {
            weights.set(i, weights.get(i) / sum);
        }
    }

    private boolean isOutOfBounds(util.Pose p) {
        return !(p.x > -67 && p.x < 67 && p.y > -67 && p.y < 67);
    }
    public double getParticleReading(util.Pose p, Lidar.Direction sensorDir) {
        double sensorHeading = MathPP.angleWrap(p.heading + sensorDir.ordinal() * Math.PI / 2, true);
        int wall = lidar.detectedWall[sensorDir.ordinal()];
        if (wall == -1) return -1;
        double relativeHeading = 0;
        double offsetFromWall = 0;
        switch (wall) {
            case 0 -> {
                offsetFromWall = 70 - p.y;
                relativeHeading = MathPP.angleWrap(sensorHeading, true);
            }
            case 1 -> {
                offsetFromWall = 70 - p.x;
                relativeHeading = MathPP.angleWrap(Math.PI/2 - sensorHeading, true);
            }
            case 2 -> {
                offsetFromWall = 70 + p.y;
                relativeHeading = MathPP.angleWrap(Math.PI - sensorHeading, true);
            }
            case 3 -> {
                offsetFromWall = 70 + p.x;
                relativeHeading = MathPP.angleWrap(3*Math.PI/2 - sensorHeading, true);
            }
        }
        double res = offsetFromWall + relativeHeading;
        if (sensorDir == Lidar.Direction.FRONT || sensorDir == Lidar.Direction.BACK) {
            res += expectedReadingOffsetLat;
        } else {
            res += expectedReadingOffsetSides;
        }
        return res;
    }
}
