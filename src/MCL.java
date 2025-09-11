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
        this.prevPose = chassis.pose;
        this.currPose = prevPose;
        initializeParticles();
    }

    private Structs.Pose redistField() {
        double FIELD_SIZE = Main.FIELD_SIZE;
        double MARGIN = Main.field.MARGIN;
        double x = Math.random() * (FIELD_SIZE - MARGIN) + MARGIN;
        double y = Math.random() * (FIELD_SIZE - MARGIN) + MARGIN;
        return SimMath.pixelsToCartesian(new Structs.Pose(x, y, Math.random() * 2 * Math.PI));
    }
    private void initializeParticles() {
        for (int i = 0; i < numParticles; i++) {
            particles.add(redistField());
            System.out.println(particles.get(i).x + ", " + particles.get(i).y + ", " + particles.get(i).heading * 180/Math.PI);
        }
    }

    public void drawParticles(Graphics g, int radius) {
        boolean ded = false;
        for (Structs.Pose p : particles) {
            Structs.Pose pixelPose = SimMath.cartesianToPixels(p);
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
        this.currPose = new Structs.Pose(this.chassis.pose.x, this.chassis.pose.y, this.chassis.pose.heading);
        this.updateX = this.currPose.x - this.prevPose.x;
        this.updateY = this.currPose.y - this.prevPose.y;
        this.updateTheta = this.currPose.heading - this.prevPose.heading;
        updateParticles();
    }

    private void updateParticles() {
        double moveDirection = Math.atan2(updateY, updateX);
        double distance = Math.hypot(updateX, updateY);
        for (Structs.Pose particle : particles) {
            double dth = updateTheta;
            particle.heading += dth;
            double noisyDistance = distance;
            double headingDelta = moveDirection - prevPose.heading;
            particle.x += noisyDistance * Math.cos(particle.heading + headingDelta);
            particle.y += noisyDistance * Math.sin(particle.heading + headingDelta);
//            if (isOutOfBounds(particle)) {
//                Structs.Pose newPose = redistField();
//                particle.x = newPose.x;
//                particle.y = newPose.y;
//                particle.heading = newPose.heading;
//            }
        }
    }

    private boolean isOutOfBounds(Structs.Pose p) {
        double FIELD_SIZE = Main.FIELD_SIZE;
        double MARGIN = Main.field.MARGIN;
        return (p.x > -70 && p.x < 70 && p.y > -70 && p.y < 70);
    }

    private void resampleParticles() {

    }

}
