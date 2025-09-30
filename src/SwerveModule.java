import util.VelocityVector;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

public class SwerveModule {
    private double angleRads = 0;
    private double speed = 0;
    private final double length;
    private final double width;
    public VelocityVector velocity = new VelocityVector(0, 0);
    public util.Pose pose = new util.Pose(0, 0, angleRads);

    public SwerveModule(double length, double width) {
        this.length = SimMath.inchesToPixels(length);
        this.width = SimMath.inchesToPixels(width);
    }

    public void setPose(util.Pose newPose) {
        this.pose = newPose;
        this.angleRads = newPose.heading;
    }
    public void updatePose(double dX, double dY) {
        pose.x += dX;
        pose.y += dY;
    }

    public void render(Graphics2D g2d) {
        g2d.setColor(Color.black);
        g2d.setStroke(new BasicStroke(3.5f));
        util.Pose newPose = SimMath.cartesianToPixels(pose);
        double x = newPose.x;
        double y = newPose.y;
        AffineTransform old = g2d.getTransform();
        g2d.translate(x, y);
        g2d.rotate(pose.heading);
        g2d.translate(-x, -y);
        Rectangle2D rect = new Rectangle2D.Double(x - width / 2, y - length / 2, width, length);
        g2d.draw(rect);
        double headingLineLength = length / 2;
        double lineX1 = x;
        double lineY1 = y - length / 2;
        double lineX2 = x;
        double lineY2 = y - length / 2 + headingLineLength;
        Line2D headingLine = new Line2D.Double(lineX1, lineY1, lineX2, lineY2);
        g2d.draw(headingLine);
    }
}
