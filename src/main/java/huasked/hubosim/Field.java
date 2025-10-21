package huasked.hubosim;

import huasked.hubosim.util.Line;
import huasked.hubosim.util.Point;
import huasked.hubosim.util.Pose;
import jdk.dynalink.linker.support.CompositeTypeBasedGuardingDynamicLinker;

import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

public class Field extends BaseRenderObject {
    public final double MARGIN;
    public final double[][] corners;
    private final ArrayList<BaseRenderObject> fieldElements = new ArrayList<>();

    public final Line[] walls;

    public Field(double width, double height, double margin) {
        this.WIDTH = inchesToPixels(width);
        this.HEIGHT = inchesToPixels(height);
        this.MARGIN = inchesToPixels(margin);
        pose = new Pose(0, 0, 0);
        corners = new double[][]
                {{MARGIN, MARGIN},
                {WIDTH - MARGIN, MARGIN},
                {WIDTH - MARGIN, HEIGHT - MARGIN},
                {MARGIN, HEIGHT - MARGIN}};
        walls = new Line[] {
                new Line(new huasked.hubosim.util.Point(corners[0][0], corners[0][1]), new huasked.hubosim.util.Point(corners[1][0], corners[1][1])), // top wall
                new Line(new huasked.hubosim.util.Point(corners[1][0], corners[1][1]), new huasked.hubosim.util.Point(corners[2][0], corners[2][1])), // right wall
                new Line(new huasked.hubosim.util.Point(corners[2][0], corners[2][1]), new huasked.hubosim.util.Point(corners[3][0], corners[3][1])), // bottom wall
                new Line(new huasked.hubosim.util.Point(corners[3][0], corners[3][1]), new Point(corners[0][0], corners[0][1]))  // left wall
        };

        this.setPreferredSize(new Dimension((int) this.WIDTH, (int) this.HEIGHT));
    }

    public void addElement(BaseRenderObject e) { fieldElements.add(e); }

    @Override
    public void update() {
        for (BaseRenderObject e : fieldElements) {
            e.update();
        }
        repaint();
    }

    @Override
    public void render(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.WHITE);
        Rectangle2D rect = new Rectangle2D.Double(0, 0, WIDTH, HEIGHT);
        g2d.fill(rect);
        drawField(g);
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        this.render(g);
        int debugX = (int) MARGIN + 10;
        int debugY = (int) MARGIN + 20;
        for (BaseRenderObject obj : fieldElements) {
            obj.render(g);
            obj.debugInfo(g, debugX, debugY);
            debugY += 20;
        }
    }

    private void drawField(Graphics g) {
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.BLACK);
        g2d.setStroke(new BasicStroke(4.0f));
        for (Line line : walls) {
            Line2D nLine = new Line2D.Double(line.start.x, line.start.y, line.end.x, line.end.y);
            g2d.draw(nLine);
        }
    }
}
