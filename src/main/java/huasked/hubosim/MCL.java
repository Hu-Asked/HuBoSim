package huasked.hubosim;

import huasked.hubosim.util.Pose;

import java.awt.*;
import java.util.ArrayList;

public class MCL {
    private final ArrayList<Pose> particles;
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
    public Pose estimatedPose;

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
        this.estimatedPose = new Pose(chassis.pose.x, chassis.pose.y, chassis.pose.heading);
        initializeParticles();
    }

    private Pose redistributeParticle() {
        double FIELD_SIZE = Main.FIELD_SIZE;
        double MARGIN = Main.field.MARGIN;
        double x = Math.random() * (FIELD_SIZE - MARGIN) + MARGIN;
        double y = Math.random() * (FIELD_SIZE - MARGIN) + MARGIN;
        return SimMath.pixelsToCartesian(new Pose(x, y, Math.random() * 2 * Math.PI));
    }

    private void initializeParticles() {
        for (int i = 0; i < numParticles; i++) {
            particles.add(redistributeParticle());
            weights.add(1.0);
            totalWeight += 1.0;
        }
    }

    public void drawParticles(Graphics g, int radius) {
        for (Pose p : particles) {
            Pose pixelPose = SimMath.cartesianToPixels(p);
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

    public void update() {
        this.prevPose = this.currPose;
        this.currPose = new Pose(this.chassis.pose.x, this.chassis.pose.y, this.chassis.pose.heading);
        this.updateX = this.currPose.x - this.prevPose.x;
        this.updateY = this.currPose.y - this.prevPose.y;
        this.updateTheta = this.currPose.heading - this.prevPose.heading;
        updateParticles();
    }

    private void updateParticles() {
        double distance = Math.hypot(updateX, updateY);
        distSinceLastUpdate += distance;
        boolean isReversing = (chassis.leftVelocity < 0 || chassis.rightVelocity < 0);
        if (isReversing) {
            distance *= -1;
        }
        this.totalWeight = 0;
        double avgX = 0;
        double avgY = 0;
        double avgTheta = 0;
        for (int i = 0; i < numParticles; i++) {
            Pose particle = particles.get(i);
            double dth = updateTheta;
            particle.heading += dth;

            double noisyDistance = distance + SimMath.getGaussianError(2.0);
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

        for (int i = 0; i < numParticles; i++) {
            Pose particle = particles.get(i);
            double weight = weighParticle(particle);
            weights.set(i, weight);
            totalWeight += weight;
            avgX += particle.x * weight;
            avgY += particle.y * weight;
            avgTheta += particle.heading * weight;
        }

        normalizeWeights();
        boolean shouldResample = distSinceLastUpdate >= minDistTravel &&
            calculateEffectiveSampleSize() < numParticles * 0.5;
        if (shouldResample) {
            resampleParticles();
            distSinceLastUpdate = 0;
        }

        this.estimatedPose = new Pose(avgX, avgY, avgTheta);
    }

    private double calculateEffectiveSampleSize() {
        double sumSquared = 0;
        for (double w : weights) {
            sumSquared += w * w;
        }
        return 1.0 / sumSquared;
    }

    private double weighParticle(Pose p) {
        double totalWeight = 1.0;
        int validReadings = 0;

        for (Lidar.Direction dir : Lidar.Direction.values()) {
            double lidarReading = lidar.distFromWall[dir.ordinal()];
            double particleReading = getParticleReading(p, dir);

            if (lidarReading < 0 || particleReading < 0) {
                continue;
            }

            double error = Math.abs(lidarReading - particleReading);

            double sigma = 5.0;
            double likelihood = 1.0 / (1.0 + (error * error) / (sigma * sigma));

            totalWeight *= likelihood;
            validReadings++;
        }

        if (validReadings == 0) {
            return 1e-10;
        }

        return Math.max(totalWeight, 1e-10);
    }

    private void normalizeWeights() {
        double sum = 0;
        for (double w : weights) {
            sum += w;
        }

        if (sum < 1e-10) {
            // Reset to uniform weights
            for (int i = 0; i < weights.size(); i++) {
                weights.set(i, 1.0 / weights.size());
            }
            totalWeight = 1.0;
        }
        else {
            for (int i = 0; i < weights.size(); i++) {
                weights.set(i, weights.get(i) / sum);
            }
            totalWeight = 1.0;
        }
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
        ArrayList<Pose> newParticles = new ArrayList<>();
        ArrayList<Double> newWeights = new ArrayList<>();

        double[] cumulativeWeights = new double[weights.size()];
        cumulativeWeights[0] = weights.get(0);
        for (int i = 1; i < weights.size(); i++) {
            cumulativeWeights[i] = cumulativeWeights[i - 1] + weights.get(i);
        }
        double segmentSize = 1.0 / numParticles;

        for (int i = 0; i < numParticles; i++) {
            double u = (Math.random() + i) * segmentSize;

            int selectedIndex = 0;
            while (selectedIndex < cumulativeWeights.length - 1 && u > cumulativeWeights[selectedIndex]) {
                selectedIndex++;
            }

            Pose original = particles.get(selectedIndex);
            double noiseX = SimMath.getGaussianError(0.5);
            double noiseY = SimMath.getGaussianError(0.5);
            double noiseTheta = SimMath.getGaussianError(0.1);

            if (Math.random() < 0.1) {
                newParticles.add(redistributeParticle());
            }
            else {
                newParticles.add(new Pose(
                    original.x + noiseX,
                    original.y + noiseY,
                    MathPP.angleWrap(original.heading + noiseTheta, true)
                ));
            }
            newWeights.add(1.0 / numParticles);
        }

        particles.clear();
        particles.addAll(newParticles);
        weights.clear();
        weights.addAll(newWeights);
        totalWeight = 1.0;
    }
}
