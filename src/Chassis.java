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

    public util.Pose pose;
    public SwerveModule[] modules = new SwerveModule[4];

    public Chassis(int w, int l, double tc, util.Pose pose) {
        this.width = SimMath.inchesToPixels(w);
        this.length = SimMath.inchesToPixels(l);
        this.turnConstant = tc;
        this.pose = pose;
    }

    public void addSwerveModules(SwerveModule leftFront, SwerveModule rightFront, SwerveModule leftBack, SwerveModule rightBack) {
        modules[0] = leftFront;
        leftBack.setPose(new util.Pose(pose.x - SimMath.pixelsToInches(width) / 2 - 1, pose.y - SimMath.pixelsToInches(length) / 2 - 1, pose.heading));
        modules[1] = rightFront;
        rightBack.setPose(new util.Pose(pose.x + SimMath.pixelsToInches(width) / 2 + 1, pose.y - SimMath.pixelsToInches(length) / 2 - 1, pose.heading));
        modules[2] = leftBack;
        leftFront.setPose(new util.Pose(pose.x - SimMath.pixelsToInches(width) / 2 - 1, pose.y + SimMath.pixelsToInches(length) / 2 + 1, pose.heading));
        modules[3] = rightBack;
        rightFront.setPose(new util.Pose(pose.x + SimMath.pixelsToInches(width) / 2 + 1, pose.y + SimMath.pixelsToInches(length) / 2 + 1, pose.heading));
    }

    public void update() {
        pose.x += lateralVelocity * -Math.cos(pose.heading + HEADINGOFFSET);
        pose.y += lateralVelocity * Math.sin(pose.heading + HEADINGOFFSET);
        for(SwerveModule module : modules) {
            module.updatePose(
                    lateralVelocity * -Math.cos(pose.heading + HEADINGOFFSET),
                    lateralVelocity * Math.sin(pose.heading + HEADINGOFFSET)
            );
        }
        pose.heading += angularVelocity;
        pose.heading %= 2 * Math.PI;
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

        for(SwerveModule module : modules) {
            if(module == null) continue;
            module.render(g2d);
        }
        g2d.setColor(Color.BLACK);
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
        double headingLineLength = 10;
        double lineX1 = x;
        double lineY1 = y - length / 2;
        double lineX2 = x;
        double lineY2 = y - length / 2 + headingLineLength;
        Line2D headingLine = new Line2D.Double(lineX1, lineY1, lineX2, lineY2);
        g2d.draw(headingLine);
    }
}
