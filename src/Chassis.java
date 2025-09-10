import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Line2D;

public class Chassis {
    final double width;
    final double length;
    double lateralVelocity = 0;
    double angularVelocity = 0;
    double leftVelocity = 0;
    double rightVelocity = 0;
    double turnConstant = 1.0;

    private static final double HEADINGOFFSET = Math.PI/2;

    public final double[] pose;

    public Chassis(int w, int l, double tc, double[] pose) {
        this.width = SimMath.inchesToPixels(w);
        this.length = SimMath.inchesToPixels(l);
        this.turnConstant = tc;
        this.pose = pose;
    }

    public double[] getVelocities() {
        return new double[]{this.leftVelocity, this.rightVelocity};
    }

    public void update() {
        pose[0] += lateralVelocity * Math.cos(pose[2] - HEADINGOFFSET);
        pose[1] += lateralVelocity * Math.sin(pose[2] - HEADINGOFFSET);
        pose[2] += angularVelocity;
        pose[2] %= 2 * Math.PI;
    }

    public void leftDrive(double velocity) {
        leftVelocity = velocity;
        updateChassisVelocity();
    }
    public void rightDrive(double velocity) {
        rightVelocity = velocity;
        updateChassisVelocity();
    }

    private void updateChassisVelocity() {
        lateralVelocity = (leftVelocity + rightVelocity) / 2;
        angularVelocity = ((rightVelocity - leftVelocity) / width) * turnConstant;
    }

    public void render(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.BLACK);
        g2d.setStroke(new BasicStroke(3.5f));

        double x = SimMath.inchesToPixels(pose[0]) + (double) Main.FIELD_SIZE / 2;
        double y = SimMath.inchesToPixels(pose[1]) + (double) Main.FIELD_SIZE / 2;

        AffineTransform old = g2d.getTransform();

        g2d.translate(x, y);
        g2d.rotate(pose[2]);
        g2d.translate(-x, -y);

        Rectangle2D rect = new Rectangle2D.Double(x - width / 2, y - length / 2, width, length);
        g2d.draw(rect);
        double headingLineLength = 10;
        double lineX1 = x;
        double lineY1 = y - length / 2;
        double lineX2 = x;
        double lineY2 = y - length / 2 + headingLineLength;
        Line2D headingLine = new Line2D.Double(lineX1, lineY1, lineX2, lineY2);
        g2d.draw(headingLine);
    }
}
