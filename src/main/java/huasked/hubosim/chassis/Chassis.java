package huasked.hubosim.chassis;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

import javax.swing.plaf.basic.BasicBorders.MarginBorder;

import huasked.hubosim.BaseRenderObject;
import huasked.hubosim.util.Pose;
import huasked.hubosim.util.VelocityVector;

public class Chassis extends BaseRenderObject {
    protected VelocityVector latVelocity = new VelocityVector(0, 0);
    protected VelocityVector angVelocity = new VelocityVector(0, 0);

    public Chassis(double x, double y, double width, double height) {
        this.pose = new Pose(x, y, 0);
        this.WIDTH = inchesToPixels(width);
        this.HEIGHT = inchesToPixels(height);
    }

    @Override
    public void update() {
        updateChassisVelocity();
        this.pose.x += latVelocity.getXComponent();
        this.pose.y += latVelocity.getYComponent();
        this.pose.heading += angVelocity.magnitude;
        pose.heading %= 2 * Math.PI;
        if(isOutOfBounds(this.pose)) {
            this.pose.x = 0;
            this.pose.y = 0;
        }
    }

    @Override
    public void render(Graphics2D g2d) {

        g2d.setColor(Color.BLACK);
        g2d.setStroke(new BasicStroke(3.5f));

        Pose newPose = cartesianToPixels(pose);
        double x = newPose.x;
        double y = newPose.y + MARGIN;

        AffineTransform old = g2d.getTransform();

        g2d.translate(x, y);
        g2d.rotate(pose.heading);
        g2d.translate(-x, -y);

        Rectangle2D rect = new Rectangle2D.Double(x - WIDTH / 2, y - HEIGHT / 2, WIDTH, HEIGHT);
        g2d.draw(rect);
        double headingLineLength = 10;
        double lineX1 = x;
        double lineY1 = y - HEIGHT / 2;
        double lineX2 = x;
        double lineY2 = y - HEIGHT / 2 + headingLineLength;
        Line2D headingLine = new Line2D.Double(lineX1, lineY1, lineX2, lineY2);
        g2d.draw(headingLine);
        g2d.setTransform(old);
    }

    private boolean isOutOfBounds(Pose p) {
        return !(p.x > -67 && p.x < 67 && p.y > -67 && p.y < 67);
    }

    protected void updateChassisVelocity() {}
}
