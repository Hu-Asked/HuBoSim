package huasked.hubosim;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

import huasked.hubosim.util.Pose;

public class FieldElements extends BaseRenderObject {
    ArrayList<Rectangle2D> elements;
    public FieldElements() {
        this.elements = new ArrayList<>();
        addElement(0, 47, 45, 5);
        addElement(0, -47, 45, 5);
    }

    public void addElement(int x, int y, int width, int height) {
        Pose pos = new Pose(x-width/2, y-height/2, 0);
        pos = cartesianToPixels(pos);
        Rectangle2D rect = new Rectangle((int) pos.x, (int) (pos.y - MARGIN), (int) inchesToPixels(width), (int) inchesToPixels(height));
        elements.add(rect);
    }

    @Override
    public void render(Graphics2D g2d) {
        g2d.setStroke(new BasicStroke(3.0f));
        g2d.setColor(Color.BLACK);
        for(Rectangle2D element : elements) {
            g2d.draw(element);
        }
    }
}
