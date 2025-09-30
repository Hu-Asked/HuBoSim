import javax.swing.*;
import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Map;

public class Field extends JPanel {
    public final double WIDTH = Main.FIELD_SIZE;
    public final double HEIGHT = Main.FIELD_SIZE;
    public final double MARGIN = 20;
    int debugX = (int)MARGIN + 10;
    int debugY = (int)MARGIN + 20;
    public final double[][] corners =
                   {{MARGIN, MARGIN},                    // top-left
                    {WIDTH - MARGIN, MARGIN},             // top-right
                    {WIDTH - MARGIN, HEIGHT - MARGIN},    // bottom-right
                    {MARGIN, HEIGHT - MARGIN}};           // bottom-left

    public final util.Line2D[] walls = {
            new util.Line2D(new util.Point(corners[0][0], corners[0][1]), new util.Point(corners[1][0], corners[1][1])), // top wall
            new util.Line2D(new util.Point(corners[1][0], corners[1][1]), new util.Point(corners[2][0], corners[2][1])), // right wall
            new util.Line2D(new util.Point(corners[2][0], corners[2][1]), new util.Point(corners[3][0], corners[3][1])), // bottom wall
            new util.Line2D(new util.Point(corners[3][0], corners[3][1]), new util.Point(corners[0][0], corners[0][1]))  // left wall
    };

    public PurePursuit pp;
    public final Chassis chassis;
    public Field(Chassis chassis, PurePursuit pp) {
        this.chassis = chassis;
        this.pp = pp;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.WHITE);
        Rectangle2D rect = new Rectangle2D.Double(0, 0, WIDTH, HEIGHT);
        g2d.fill(rect);
        drawField(g);
//        drawPath(g, Main.chosenPath, Color.CYAN);
//        drawPath(g, pp.actualPath, Color.GREEN);
//        drawLookaheadCircle(g, pp.lookaheadDistance);
//        Main.lidar.drawSensorLines(g);
//        Main.mcl.drawParticles(g, 6);
        chassis.render(g);
    }
    public void drawPath(Graphics g, ArrayList<Map.Entry<util.Point, Double>> path, Color color) {
        if (path == null || path.isEmpty()) return;

        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(color);
        g2d.setStroke(new BasicStroke(2.0f));
        for (int i = 0; i < path.size() - 1; i++) {
            util.Point start = path.get(i).getKey();
            util.Point end = path.get(i + 1).getKey();
            Line2D line = new Line2D.Double(
                    SimMath.inchesToPixels(start.x) + WIDTH / 2,
                    SimMath.inchesToPixels(-start.y) + HEIGHT / 2,
                    SimMath.inchesToPixels(end.x) + WIDTH / 2,
                    SimMath.inchesToPixels(-end.y) + HEIGHT / 2
            );
            g2d.draw(line);
        }
    }
    public void updateField() {
        constrainChassis();
        chassis.update();
        repaint();
    }
    private void constrainChassis() {
        util.Pose pose = chassis.pose;
        double halfWidth = SimMath.pixelsToInches(chassis.width) / 2;
        double halfLength = SimMath.pixelsToInches(chassis.length) / 2;
        if(pose.x - halfWidth < - 70) {
            pose.x = -70 + halfWidth;
        } else if(pose.x + halfWidth > 70) {
            pose.x = 70 - halfWidth;
        }

        if(pose.y - halfLength < -70) {
            pose.y = -70 + halfLength;
        } else if(pose.y + halfLength > 70) {
            pose.y = 70 - halfLength;
        }

    }
    public void drawField(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.BLACK);
        g2d.setStroke(new BasicStroke(4.0f));

        for (util.Line2D line : walls) {
            Line2D nLine = new Line2D.Double(line.start.x, line.start.y, line.end.x, line.end.y);
            g2d.draw(nLine);
        }
        util.Pose robPose = SimMath.cartesianToPixels(chassis.pose);
        double robotX = inchesToScreenX(chassis.pose.x);
        double robotY = inchesToScreenY(chassis.pose.y);
        double robotDotRadius = 8;
        g2d.setColor(Color.BLUE);
        g2d.fillOval(
                (int)(robotX - robotDotRadius),
                (int)(robotY - robotDotRadius),
                (int)(robotDotRadius * 2),
                (int)(robotDotRadius * 2)
        );
        util.Point target = pp.targetPoint;
        g2d.drawString(
                String.format("Target: (%.2f, %.2f)", target.x, target.y),
                debugX, debugY
        );
        g2d.drawString(
                String.format("Pose: (%.2f, %.2f, %.2f)", chassis.pose.x, chassis.pose.y, chassis.pose.heading * 180/Math.PI),
                debugX, debugY + 20);
        g2d.drawString(
                String.format("Relative Heading: %.2f . Abs Heading: %.2f", pp.rHeading, pp.aHeading),
                debugX, debugY + 40);
        g2d.drawString(
                String.format("Segment: %d . Scalar: %.2f . Progress: %.2f%%", pp.pathSegIndex, pp.pathSegmentScalarProgression, pp.pathFollowPercentage),
                debugX, debugY + 60);
    }
    public void drawLookaheadCircle(Graphics g, double lookaheadDistance) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(new Color(0, 180, 255, 120)); // Light blue with transparency
        g2d.setStroke(new BasicStroke(1.5f));

        // Convert lookahead distance from inches to pixels
        double radiusPixels = SimMath.inchesToPixels(lookaheadDistance);

        // Center the circle on the robot's position
        util.Pose center = SimMath.cartesianToPixels(chassis.pose);
        double centerX = center.x;
        double centerY = center.y;

        // Draw the circle
        g2d.drawOval(
            (int)(centerX - radiusPixels),
            (int)(centerY - radiusPixels),
            (int)(radiusPixels * 2),
            (int)(radiusPixels * 2)
        );
    }

    public double inchesToScreenX(double inches) {
        return SimMath.inchesToPixels(inches) + WIDTH/2;
    }

    public double inchesToScreenY(double inches) {
        return HEIGHT / 2 - SimMath.inchesToPixels(inches);
    }
}
