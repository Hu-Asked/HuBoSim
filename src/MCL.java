import Structs.Pose;

import java.awt.*;
import java.sql.Struct;
import java.util.ArrayList;

public class MCL {
    private ArrayList<Structs.Pose> particles;
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
        this.prevPose = new Pose(chassis.pose[0], chassis.pose[1], chassis.pose[2]);
        this.currPose = prevPose;
        initializeParticles();
    }

    private void initializeParticles() {
        for (int i = 0; i < numParticles; i++) {
            double FIELD_SIZE = Main.FIELD_SIZE;
            double MARGIN = Main.field.MARGIN;
            double x = SimMath.pixelsToInches(Math.random() * (FIELD_SIZE - MARGIN) + MARGIN);
            double y = SimMath.pixelsToInches(Math.random() * (FIELD_SIZE - MARGIN) + MARGIN);
            particles.add(new Structs.Pose(x, y, Math.random() * 2 * Math.PI));
        }
    }

    public void drawParticles(Graphics g, int radius) {
        for (Structs.Pose p : particles) {
            g.setColor(Color.BLUE);
            int px = (int) SimMath.inchesToPixels(p.x);
            int py = (int) SimMath.inchesToPixels(p.y);
            g.fillOval(px - radius / 2, py - radius / 2, radius, radius);
            //draw line representing heading
            g.setColor(Color.BLACK);
            int hx = (int) (px + radius * Math.cos(p.heading - Math.PI / 2));
            int hy = (int) (py + radius * Math.sin(p.heading - Math.PI / 2));
            g.drawLine(px, py, hx, hy);
        }
    }

    public void update() {
        this.prevPose = currPose;
        this.currPose = new Pose(chassis.pose[0], chassis.pose[1], chassis.pose[2]);
        this.updateX = this.currPose.x - this.prevPose.x;
        this.updateY = this.currPose.y - this.prevPose.y;
        this.updateTheta = currPose.heading - prevPose.heading;
        updateParticles();
    }

    private void updateParticles() {
        double moveDirection = Math.atan2(updateY, updateX);
        double distance = Math.hypot(updateX, updateY);
        for (Structs.Pose particle : particles) {
            double dth = updateTheta * (1.0 + SimMath.randomGaussian() / 100.0);
            particle.heading += dth;
            double noisyDistance = distance * (1.0 + SimMath.randomGaussian() / 100.0);
            double headingDelta = moveDirection - prevPose.heading;
            particle.x += noisyDistance * Math.cos(particle.heading + headingDelta);
            particle.y += noisyDistance * Math.sin(particle.heading + headingDelta);
        }
    }

    private boolean isOutOfBounds(Structs.Pose p) {
        double FIELD_SIZE = Main.FIELD_SIZE;
        double MARGIN = Main.field.MARGIN;
        return p.x < SimMath.pixelsToInches(MARGIN) || p.x > SimMath.pixelsToInches(FIELD_SIZE - MARGIN) ||
               p.y < SimMath.pixelsToInches(MARGIN) || p.y > SimMath.pixelsToInches(FIELD_SIZE - MARGIN);
    }
    private void resampleParticles() {

    }

}
