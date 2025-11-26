package huasked.hubosim;

import java.awt.Graphics;
import java.awt.Graphics2D;

import huasked.hubosim.util.Pose;

public abstract class BaseRenderObject extends Base {
    public Pose pose;
    protected double WIDTH;
    protected double HEIGHT;

    public void update() {}

    public void render(Graphics2D g2d) {}

    public void debugInfo(Graphics g, int x, int y) { }
}
