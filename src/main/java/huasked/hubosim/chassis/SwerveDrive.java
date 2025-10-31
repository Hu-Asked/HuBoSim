package huasked.hubosim.chassis;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

import huasked.hubosim.algorithm.MathPP;
import huasked.hubosim.util.Pose;
import huasked.hubosim.util.VelocityVector;

public class SwerveDrive extends Chassis {
    private SwerveModule[] swerveModules;
    public SwerveDrive(int x, int y, int width, int height, SwerveModule[] swerveModules) {
        super(x, y, width, height);
        this.swerveModules = swerveModules;
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
        for (SwerveModule module : swerveModules) {
            if (module == null) {
                continue;
            }
            module.render(g);
        }
        g2d.setTransform(old);
    }

    @Override
    public void update() {
        for (SwerveModule module : swerveModules) {
            module.parentPose = this.pose;
        }
        super.update();
        positionSwerveModules(false);
    }

    @Override
    protected void updateChassisVelocity() {
        this.latVelocity = sumSwerveVelocity();
        this.angVelocity = getAngularVel();
    }
    
    public void addSwerveModules(SwerveModule[] swerveModules) {
        for (SwerveModule module : swerveModules) {
            module.parentPose = this.pose;
        }
        this.swerveModules = swerveModules;
        positionSwerveModules(true);
    }

    public void positionSwerveModules(boolean useRobotHeading) {
        this.swerveModules[2].setPose(new Pose(this.pose.x - pixelsToInches(this.WIDTH) / 2 - 1, this.pose.y - pixelsToInches(this.HEIGHT) / 2 - 1,
            useRobotHeading ? this.pose.heading : this.swerveModules[2].pose.heading));
        this.swerveModules[3].setPose(new Pose(this.pose.x + pixelsToInches(this.WIDTH) / 2 + 1, this.pose.y - pixelsToInches(this.HEIGHT) / 2 - 1,
            useRobotHeading ? this.pose.heading : this.swerveModules[3].pose.heading));
        this.swerveModules[0].setPose(new Pose(this.pose.x - pixelsToInches(this.WIDTH) / 2 - 1, this.pose.y + pixelsToInches(this.HEIGHT) / 2 + 1,
            useRobotHeading ? this.pose.heading : this.swerveModules[0].pose.heading));
        this.swerveModules[1].setPose(new Pose(this.pose.x + pixelsToInches(this.WIDTH) / 2 + 1, this.pose.y + pixelsToInches(this.HEIGHT) / 2 + 1,
            useRobotHeading ? this.pose.heading : this.swerveModules[1].pose.heading));
    }

    private void normalizeSpeeds() {
        double maxSpeed = 3.0;
        double highest = 0;
        for (SwerveModule module : swerveModules) {
            highest = Math.max(highest, module.velocity.magnitude);
        }
        if (highest > maxSpeed) {
            double ratio = maxSpeed / highest;
            for (SwerveModule module : swerveModules) {
                module.velocity.magnitude *= ratio;
            }
        }
    }

    private VelocityVector sumSwerveVelocity() {
        normalizeSpeeds();
        double sumX = 0;
        double sumY = 0;
        for (SwerveModule module : swerveModules) {
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

        for (SwerveModule module : swerveModules) {
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
        } else {
            this.swerveModules[0].setAngleRads(frontLeftAngle);
            this.swerveModules[1].setAngleRads(frontRightAngle);
            this.swerveModules[2].setAngleRads(backLeftAngle);
            this.swerveModules[3].setAngleRads(backRightAngle);
        }
        this.swerveModules[0].setSpeed(frontLeftSpeed);
        this.swerveModules[1].setSpeed(frontRightSpeed);
        this.swerveModules[2].setSpeed(backLeftSpeed);
        this.swerveModules[3].setSpeed(backRightSpeed);
    }
}
