package huasked.hubosim.control;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

import huasked.hubosim.BaseRenderObject;
import huasked.hubosim.util.Pose;

public class MouseController extends BaseRenderObject implements MouseListener {
    public Pose pose = new Pose(0, 0, 0);
    public boolean newPress = false;
    @Override
    public void mouseClicked(MouseEvent arg0) {
        pose.x = arg0.getPoint().x;
        pose.y = arg0.getPoint().y;
        pose = pixelsToCartesian(pose);
        pose.heading = rd.nextDouble() < 0.5 ? -1 : 1;
        newPress = true;
    }

    @Override
    public void debugInfo(Graphics g, int x, int y) {
        String output = String.format("%.2f %.2f %.2f", pose.x, pose.y, pose.heading);
        g.drawString(output, x, y);
    }



    @Override
    public void mouseEntered(MouseEvent arg0) {
    }

    @Override
    public void mouseExited(MouseEvent arg0) {
    }

    @Override
    public void mousePressed(MouseEvent arg0) {
    }

    @Override
    public void mouseReleased(MouseEvent arg0) {
    }
    
}
