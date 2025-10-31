package huasked.hubosim;

import huasked.hubosim.util.Pose;

import java.awt.*;

public abstract class BaseRenderObject extends Base {
    public Pose pose;
    protected double WIDTH;
    protected double HEIGHT;

    public void update() {}

    public void render(Graphics g) {}


    public void debugInfo(Graphics g, int x, int y) { }
}
