package huasked.hubosim;

import huasked.hubosim.util.Pose;
import huasked.hubosim.util.VelocityVector;

import java.awt.*;

public class Chassis extends BaseRenderObject {
    private VelocityVector latVelocity = new VelocityVector(0, 0);
    private VelocityVector angVelocity = new VelocityVector(0, 0);
    public Chassis(double x, double y, double width, double height) {
        this.pose = new Pose(x, y, 0);
        this.WIDTH = width;
        this.HEIGHT = height;
    }

    @Override
    public void update() {
        this.pose.x += latVelocity.getXComponent();
        this.pose.y += angVelocity.getYComponent();
    }

    @Override
    public void render(Graphics g) {

    }
    public void updateVelocities(VelocityVector latVel, VelocityVector angVel) {
        this.latVelocity = latVel;
        this.angVelocity = angVel;
    }
}
