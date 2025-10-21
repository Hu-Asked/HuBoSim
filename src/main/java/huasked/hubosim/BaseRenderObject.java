package huasked.hubosim;

import huasked.hubosim.util.Pose;

import javax.swing.*;
import java.awt.*;

public abstract class BaseRenderObject extends JPanel {
    protected static final double inchesToPixelsRatio = 140.0/880.0;
    public Pose pose;
    protected double WIDTH;
    protected double HEIGHT;

    public void update() {}

    public void render(Graphics g) {}

    protected static double inchesToPixels(double inches) { return inches / inchesToPixelsRatio; }

    protected static double pixelsToInches(double pixels) { return pixels * inchesToPixelsRatio; }

    protected static Pose pixelsToCartesian(Pose pose) { return new Pose(pixelsToInches(pose.x) - 70, -pixelsToInches(pose.y) + 70, pose.heading); }

    protected static Pose cartesianToPixels(Pose pose) { return new Pose(inchesToPixels(pose.x + 70), -inchesToPixels(pose.y - 70), pose.heading); }

    public void debugInfo(Graphics g, int x, int y) { }
}
