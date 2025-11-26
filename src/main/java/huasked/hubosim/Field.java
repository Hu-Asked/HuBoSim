package huasked.hubosim;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

import huasked.hubosim.util.Line;
import huasked.hubosim.util.Point;
import huasked.hubosim.util.Pose;

public class Field extends BaseRenderObject {
    public final double[][] corners;
    private final ArrayList<BaseRenderObject> fieldElements = new ArrayList<>();

    public final Line[] walls;

    public Field(double width, double height) {
        this.WIDTH = inchesToPixels(width);
        this.HEIGHT = inchesToPixels(height);
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
    public void render(Graphics2D g2d) {
        g2d.setColor(Color.WHITE);
        Rectangle2D rect = new Rectangle2D.Double(0, 0, WIDTH, HEIGHT);
        g2d.fill(rect);
        // PPPaths.renderPath(g, PPPaths.activePath, Color.CYAN, this.WIDTH, this.HEIGHT);
        // PPPaths.renderPath(g, Main.pp.actualPath, Color.GREEN, this.WIDTH, this.HEIGHT);
        drawField(g2d);
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        this.render((Graphics2D) g);
        int debugX = (int) MARGIN + 10;
        int debugY = (int) MARGIN + 20;
        Graphics2D g2d = (Graphics2D) g;
        for (BaseRenderObject obj : fieldElements) {
            AffineTransform old = g2d.getTransform();
            obj.render(g2d);
            obj.debugInfo(g, debugX, debugY);
            debugY += 20;
            g2d.setTransform(old);
        }
    }

    private void drawField(Graphics2D g2d) {
        g2d.setColor(Color.BLACK);
        g2d.setStroke(new BasicStroke(4.0f));
        for (Line line : walls) {
            Line2D nLine = new Line2D.Double(line.start.x, line.start.y, line.end.x, line.end.y);
            g2d.draw(nLine);
        }
    }
}
