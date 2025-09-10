import javax.sound.sampled.Line;
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

    public final Structs.Line2D[] walls = {
            new Structs.Line2D(new Structs.Point(corners[0][0], corners[0][1]), new Structs.Point(corners[1][0], corners[1][1])), // top wall
            new Structs.Line2D(new Structs.Point(corners[1][0], corners[1][1]), new Structs.Point(corners[2][0], corners[2][1])), // right wall
            new Structs.Line2D(new Structs.Point(corners[2][0], corners[2][1]), new Structs.Point(corners[3][0], corners[3][1])), // bottom wall
            new Structs.Line2D(new Structs.Point(corners[3][0], corners[3][1]), new Structs.Point(corners[0][0], corners[0][1]))  // left wall
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
        drawPath(g, Main.chosenPath, Color.BLUE);
        drawPath(g, pp.actualPath, Color.GREEN);
        drawLookaheadCircle(g, pp.lookaheadDistance);
        Main.lidar.drawSensorLines(g);
        chassis.render((Graphics2D) g);
    }
    // Updated method that accepts a Graphics parameter
    public void drawPath(Graphics g, ArrayList<Map.Entry<Structs.Point, Double>> path, Color color) {
        if (path == null || path.isEmpty()) return;

        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(color);
        g2d.setStroke(new BasicStroke(2.0f));
        for (int i = 0; i < path.size() - 1; i++) {
            Structs.Point start = path.get(i).getKey();
            Structs.Point end = path.get(i + 1).getKey();
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
        double[] pose = chassis.pose;
        double halfWidth = SimMath.pixelsToInches(chassis.width) / 2;
        double halfLength = SimMath.pixelsToInches(chassis.length) / 2;
        if(pose[0] - halfWidth < - 70) {
            pose[0] = -70 + halfWidth;
        } else if(pose[0] + halfWidth > 70) {
            pose[0] = 70 - halfWidth;
        }

        if(pose[1] - halfLength < -70) {
            pose[1] = -70 + halfLength;
        } else if(pose[1] + halfLength > 70) {
            pose[1] = 70 - halfLength;
        }

    }
    public void drawField(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.BLACK);
        g2d.setStroke(new BasicStroke(4.0f));

        for (Structs.Line2D line : walls) {
            Line2D nLine = new Line2D.Double(line.start.x, line.start.y, line.end.x, line.end.y);
            g2d.draw(nLine);
        }
        if (pp != null && pp.targetPoint != null) {
            Structs.Point target = pp.targetPoint;
            double dotRadius = 10; // Adjust for desired size
            double centerX = inchesToScreenX(target.x);
            double centerY = inchesToScreenY(target.y);
            g2d.setColor(Color.BLUE);
            g2d.fillOval(
                (int)(centerX - dotRadius),
                (int)(centerY - dotRadius),
                (int)(dotRadius * 2),
                (int)(dotRadius * 2)
            );
        }
        double robotX = inchesToScreenX(chassis.pose[0]);
        double robotY = inchesToScreenY(-chassis.pose[1]);
        double robotDotRadius = 8;
        g2d.setColor(Color.BLUE);
        g2d.fillOval(
                (int)(robotX - robotDotRadius),
                (int)(robotY - robotDotRadius),
                (int)(robotDotRadius * 2),
                (int)(robotDotRadius * 2)
        );
        Structs.Point target = pp.targetPoint;
        g2d.drawString(
                String.format("Target: (%.2f, %.2f)", target.x, target.y),
                debugX, debugY
        );
        g2d.drawString(
                String.format("Pose: (%.2f, %.2f, %.2f)", chassis.pose[0], -chassis.pose[1], chassis.pose[2] * 180/Math.PI),
                debugX, debugY + 20);
        g2d.drawString(
                String.format("PP Pose: (%.2f, %.2f, %.2f)", pp.currentPose.x, pp.currentPose.y, pp.currentPose.heading * 180/Math.PI),
                debugX, debugY + 40);
        g2d.drawString(
                String.format("Relative Heading: %.2f . Abs Heading: %.2f", pp.rHeading, pp.aHeading),
                debugX, debugY + 60);
        g2d.drawString(
                String.format("Segment: %d . Scalar: %.2f . Progress: %.2f%%", pp.pathSegIndex, pp.pathSegmentScalarProgression, pp.pathFollowPercentage),
                debugX, debugY + 80);
        g2d.drawString(
                String.format("Left: %.2f | Right: %.2f | Front: %.2f | Back: %.2f", Main.lidar.distFromWall[0], Main.lidar.distFromWall[1], Main.lidar.distFromWall[2], Main.lidar.distFromWall[3]),
                debugX, debugY + 100);
    }
    public void drawLookaheadCircle(Graphics g, double lookaheadDistance) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(new Color(0, 180, 255, 120)); // Light blue with transparency
        g2d.setStroke(new BasicStroke(1.5f));

        // Convert lookahead distance from inches to pixels
        double radiusPixels = SimMath.inchesToPixels(lookaheadDistance);

        // Center the circle on the robot's position
        double centerX = inchesToScreenX(chassis.pose[0]);
        double centerY = inchesToScreenY(-chassis.pose[1]);

        // Draw the circle
        g2d.drawOval(
            (int)(centerX - radiusPixels),
            (int)(centerY - radiusPixels),
            (int)(radiusPixels * 2),
            (int)(radiusPixels * 2)
        );
    }

    // Helper methods needed for coordinate conversion
    private double inchesToScreenX(double inches) {
        return SimMath.inchesToPixels(inches) + WIDTH/2;
    }

    private double inchesToScreenY(double inches) {
        // Flip Y-axis so positive Y points upward
        return HEIGHT / 2 - SimMath.inchesToPixels(inches);
    }
}
