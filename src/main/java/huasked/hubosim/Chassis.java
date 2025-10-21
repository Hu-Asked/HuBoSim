package huasked.hubosim;

import huasked.hubosim.util.Pose;
import huasked.hubosim.util.VelocityVector;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

public class Chassis extends BaseRenderObject {
    private VelocityVector latVelocity = new VelocityVector(0, 0);
    private VelocityVector angVelocity = new VelocityVector(0, 0);
    private SwerveModule[] modules = new SwerveModule[4];
    public Chassis(double x, double y, double WIDTH, double height) {
        this.pose = new Pose(x, y, 0);
        this.WIDTH = inchesToPixels(WIDTH);
        this.HEIGHT = inchesToPixels(height);
    }

    @Override
    public void update() {
        for (SwerveModule module : modules) {
            module.parentPose = this.pose;
        }
        updateChassisVelocity();
        this.pose.x += latVelocity.getXComponent();
        this.pose.y += angVelocity.getYComponent();
        positionSwerveModules(false);
        this.pose.heading += angVelocity.magnitude;
        pose.heading %= 2 * Math.PI;
    }

    @Override
    public void render(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;

        g2d.setColor(Color.BLACK);
        g2d.setStroke(new BasicStroke(3.5f));

        Pose newPose = cartesianToPixels(pose);
        double x = newPose.x;
        double y = newPose.y;

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
        for (SwerveModule module : modules) {
            if (module == null) {
                continue;
            }
            module.render(g);
        }
        g2d.setTransform(old);
    }

    public void updateDrive(double leftX, double leftY, double rightX, double rightY) {
        double L = pixelsToInches(this.HEIGHT);
        double W = pixelsToInches(this.WIDTH);
        double R = Math.hypot(L, W);
        double targetHeading = this.pose.heading;
        if (Math.sqrt(rightY * rightY + rightX * rightX) > 0.5) {
            targetHeading = -(Math.atan2(rightY, rightX) - Math.PI / 2);
        }
        double delta = MathPP.angleWrap(targetHeading - MathPP.angleWrap(this.pose.heading, true), true);
        double turnMulti = 0.75;
        double angularMag = delta * turnMulti;
        double a = leftX - angularMag * (L / R);
        double b = leftX + angularMag * (L / R);
        double c = leftY - angularMag * (W / R);
        double d = leftY + angularMag * (W / R);

        double backRightSpeed = Math.hypot(a, d);
        double backLeftSpeed = Math.hypot(a, c);
        double frontRightSpeed = Math.hypot(b, d);
        double frontLeftSpeed = Math.hypot(b, c);

        double backRightAngle = Math.atan2(a, c);
        double backLeftAngle = Math.atan2(a, d);
        double frontRightAngle = Math.atan2(b, c);
        double frontLeftAngle = Math.atan2(b, d);
        if (Math.abs(leftX) <= 2e-2 && Math.abs(leftY) <= 2e-2 && Math.abs(rightX) <= 2e-2) {
            frontLeftSpeed = 0;
            frontRightSpeed = 0;
            backLeftSpeed = 0;
            backRightSpeed = 0;
        }
        else {
            this.modules[0].setAngleRads(frontLeftAngle);
            this.modules[1].setAngleRads(frontRightAngle);
            this.modules[2].setAngleRads(backLeftAngle);
            this.modules[3].setAngleRads(backRightAngle);
        }
        this.modules[0].setSpeed(frontLeftSpeed);
        this.modules[1].setSpeed(frontRightSpeed);
        this.modules[2].setSpeed(backLeftSpeed);
        this.modules[3].setSpeed(backRightSpeed);
    }

    public void addSwerveModules(SwerveModule leftFront, SwerveModule rightFront, SwerveModule leftBack, SwerveModule rightBack) {
        modules[0] = leftFront;
        modules[1] = rightFront;
        modules[2] = leftBack;
        modules[3] = rightBack;
        for (SwerveModule module : modules) {
            module.parentPose = this.pose;
        }
        positionSwerveModules(true);
    }

    public void positionSwerveModules(boolean useRobotHeading) {
        modules[2].setPose(new Pose(pose.x - pixelsToInches(this.WIDTH) / 2 - 1, pose.y - pixelsToInches(HEIGHT) / 2 - 1,
            useRobotHeading ? pose.heading : modules[2].pose.heading));
        modules[3].setPose(new Pose(pose.x + pixelsToInches(WIDTH) / 2 + 1, pose.y - pixelsToInches(HEIGHT) / 2 - 1,
            useRobotHeading ? pose.heading : modules[3].pose.heading));
        modules[0].setPose(new Pose(pose.x - pixelsToInches(WIDTH) / 2 - 1, pose.y + pixelsToInches(HEIGHT) / 2 + 1,
            useRobotHeading ? pose.heading : modules[0].pose.heading));
        modules[1].setPose(new Pose(pose.x + pixelsToInches(WIDTH) / 2 + 1, pose.y + pixelsToInches(HEIGHT) / 2 + 1,
            useRobotHeading ? pose.heading : modules[1].pose.heading));
    }


    private void normalizeSpeeds() {
        double maxSpeed = 3.0;
        double highest = 0;
        for (SwerveModule module : modules) {
            highest = Math.max(highest, module.velocity.magnitude);
        }
        if (highest > maxSpeed) {
            double ratio = maxSpeed / highest;
            for (SwerveModule module : modules) {
                module.velocity.magnitude *= ratio;
            }
        }
    }

    private VelocityVector sumSwerveVelocity() {
        normalizeSpeeds();
        double sumX = 0;
        double sumY = 0;
        for (SwerveModule module : modules) {
            VelocityVector vel = new VelocityVector(module.velocity.magnitude, module.velocity.getDirection());
            vel.setDirection(module.velocity.getDirection());
            vel.magnitude *= 0.25;
            sumX += vel.getXComponent();
            sumY += vel.getYComponent();
        }
        double magnitude = Math.hypot(sumX, sumY);
        double angle = Math.atan2(sumY, sumX);
        VelocityVector velVec = new VelocityVector(magnitude, angle);
        return velVec;
    }

    private VelocityVector getAngularVel() {
        double angularVelocitySum = 0;
        double totalWeight = 0;
        double centerX = pose.x;
        double centerY = pose.y;

        for (SwerveModule module : modules) {
            if (module == null) {
                continue;
            }

            double dx = module.pose.x - centerX;
            double dy = module.pose.y - centerY;
            double rSquared = dx * dx + dy * dy;
            if (rSquared == 0) {
                continue;
            }

            double vx = module.velocity.getXComponent();
            double vy = module.velocity.getYComponent();

            double tangential = (vx * -dy + vy * dx) / Math.sqrt(rSquared);
            double angular = tangential / Math.sqrt(rSquared);
            angularVelocitySum += angular;
            totalWeight += 1;
        }
        angularVelocitySum *= 0.25;
        return new VelocityVector(totalWeight > 0 ? angularVelocitySum / totalWeight : 0, 0);
    }
    @Override
    public void debugInfo(Graphics g, int x, int y) {
    }
    private void updateChassisVelocity() {
        this.latVelocity = sumSwerveVelocity();
        this.angVelocity = getAngularVel();
    }
}
