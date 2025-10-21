package huasked.hubosim;

import huasked.hubosim.util.Pose;
import huasked.hubosim.util.VelocityVector;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

public class SwerveModule extends BaseRenderObject {
    public VelocityVector velocity = new VelocityVector(0, 0);
    public Pose parentPose = new Pose(0, 0, 0);
    public SwerveModule(double HEIGHT, double WIDTH) {
        this.HEIGHT = inchesToPixels(HEIGHT);
        this.WIDTH  = inchesToPixels(WIDTH);
        this.pose = new Pose(0, 0, 0);
    }

    public void setAngleRads(double angleRads) {
        this.pose.heading = angleRads;
    }

    public void setSpeed(double speed) {
        this.velocity.magnitude = speed;
        this.velocity.setDirection(-this.pose.heading);
    }

    public void setPose(Pose newPose) {
        this.pose = newPose;
    }

    @Override
    public void render(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.black);
        g2d.setStroke(new BasicStroke(3.5f));
        Pose newPose = cartesianToPixels(pose);
        double x = newPose.x;
        double y = newPose.y;
        AffineTransform old = g2d.getTransform();
        g2d.translate(x, y);
        g2d.rotate(pose.heading - parentPose.heading);
        g2d.translate(-x, -y);
        Rectangle2D rect = new Rectangle2D.Double(x - WIDTH / 2, y - HEIGHT / 2, WIDTH, HEIGHT);
        g2d.draw(rect);
        double headingLineLength = HEIGHT / 2;
        double lineX1 = x;
        double lineY1 = y - HEIGHT / 2;
        double lineX2 = x;
        double lineY2 = y - HEIGHT / 2 + headingLineLength;
        Line2D headingLine = new Line2D.Double(lineX1, lineY1, lineX2, lineY2);
        g2d.draw(headingLine);
        g2d.setTransform(old);
    }
}
