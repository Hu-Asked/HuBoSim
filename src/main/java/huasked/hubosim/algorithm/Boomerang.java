package huasked.hubosim.algorithm;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import huasked.hubosim.BaseRenderObject;
import huasked.hubosim.Main;
import huasked.hubosim.chassis.TankDrive;
import huasked.hubosim.util.Pose;

public class Boomerang extends BaseRenderObject {
    public Pose pose;
    public double kP;
    public double kPt;
    public double trackWidth;
    public TankDrive chassis;
    private Pose carrot = new Pose(0, 0, 0);
    private Pose target = new Pose(0, 0, 0);
    public Boomerang(double kP, double kPt, double trackWidth, TankDrive chassis) {
        this.kP = kP;
        this.kPt = kPt;
        this.trackWidth = trackWidth;
        this.chassis = chassis;
    }

    private double normalizeAngle(double heading) {
        return ((heading % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
    }   


    private double angleError(double target, double current) {
        target = normalizeAngle(target);
        current = normalizeAngle(current);
        return toRad(Math.IEEEremainder(toDeg(target - current), toDeg(2 * Math.PI)));
    }

    public void moveTo(Pose t, double lead, double earlyExit) {
        target = new Pose(t.x, t.y, t.heading);

        boolean settling = false;

        while(true) {
            Pose cur = new Pose(this.pose.x, this.pose.y, this.pose.heading);

            double latError = Math.hypot(cur.x - target.x, cur.y - target.y);
            if(latError < earlyExit) break;
            if(latError < 7.5) settling = true;

            // https://www.desmos.com/calculator/sptjw5szex
            double xc = target.x - latError * Math.sin(target.heading) * lead;
            double yc = target.y - latError * Math.cos(target.heading) * lead;
            carrot = new Pose(xc, yc, 0);
            if (settling) carrot = target;  
            latError = Math.hypot(cur.x - carrot.x, cur.y - carrot.y);

            double diff = normalizeAngle(Math.atan2(carrot.x - cur.x, carrot.y - cur.y) + Math.PI);
            double angError = angleError(cur.heading, diff);
        
            if (settling) latError *= -Math.cos(angError);
            else latError *= Math.cos(angleError(cur.heading, diff)) < 0 ? 1 : -1;
            
            double latOut = Math.clamp(kP * latError, -1, 1);
            double angOut = Math.clamp(kPt * toDeg(angError), -1, 1);

            double radius = 1 / Math.abs(getCurve(carrot));
            double maxSlipSpeed = Math.sqrt(8 * radius * 9.8 * 0.01);
            latOut = Math.clamp(latOut, -maxSlipSpeed, maxSlipSpeed);
            double overturn = Math.abs(angOut) + Math.abs(latOut) - 2;
            if (overturn > 0) latOut -= latOut > 0 ? overturn : -overturn;

            double left = latOut - angOut;
            double right = latOut + angOut;
            double ratio = Math.max(Math.abs(left), Math.abs(right)) / 2;
            if (ratio > 1) {
                left /= ratio;
                right /= ratio;
            }
            chassis.leftDrive(left);
            chassis.rightDrive(right);
            Main.tick();
        }
    }

    @Override
    public void render(Graphics2D g2d) {
        Pose t = cartesianToPixels(target);
        Pose c = cartesianToPixels(carrot);
        g2d.setColor(Color.RED);
        g2d.fillOval((int)(t.x - 2), (int)(t.y - 2), 10, 10);
        
        g2d.setColor(Color.GREEN);
        g2d.fillOval((int)(c.x - 2), (int)(c.y - 2), 10, 10);
    }
    private double getCurve(Pose t) {
        double side = (Math.sin(pose.heading) * (t.x - pose.x) - Math.cos(pose.heading) * (t.y - pose.y)) >= 0 ? 1 : -1;
        // calculate center point and radius
        double a = -Math.tan(pose.heading);
        double c = Math.tan(pose.heading) * pose.x - pose.y;
        double x = Math.abs(a * t.x + t.y + c) / Math.sqrt((a * a) + 1);
        double d = Math.hypot(t.x - pose.x, t.y - pose.y);

        // return curvature
        return side * ((2 * x) / (d * d));

    }
}
