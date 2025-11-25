package huasked.hubosim.chassis;

import java.awt.Graphics;

import huasked.hubosim.util.VelocityVector;
public class TankDrive extends Chassis {
    private double leftVelocity = 0;
    private double rightVelocity = 0;
    private double turnConstant = 1.0;

    @Override
    protected void updateChassisVelocity() {
        this.latVelocity = new VelocityVector((leftVelocity + rightVelocity) / 2, -this.pose.heading - Math.PI);
        this.angVelocity = new VelocityVector((rightVelocity - leftVelocity)/this.WIDTH * turnConstant, 0);
    }

    public TankDrive(double x, double y, double width, double height, double turnConstant) {
        super(x, y, width, height);
        this.turnConstant = turnConstant;
    }

    public void updateDrive(double leftX, double leftY) {
        leftDrive(leftY - leftX);
        rightDrive(leftY + leftX);
    }
    public void leftDrive(double vel) {
        this.leftVelocity = vel;
    }

    public void rightDrive(double vel) {
        this.rightVelocity = vel;
    }

    @Override
    public void debugInfo(Graphics g, int x, int y) {
        String output = String.format("%.2f, %.2f, %.2f", this.pose.x, this.pose.y, this.pose.heading);
        g.drawString(output, x, y);
    }

}
