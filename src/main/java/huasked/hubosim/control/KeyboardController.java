package huasked.hubosim.control;

import huasked.hubosim.Main;
import huasked.hubosim.SimMath;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.HashMap;

public class KeyboardController implements KeyListener {
    private HashMap<Integer, Boolean> keyStates = new HashMap<>();
    double speed = 1;

    private void updateDrive() {
        double leftY = (keyStates.containsKey(KeyEvent.VK_W) && keyStates.get(KeyEvent.VK_W) ? speed : 0) +
            (keyStates.containsKey(KeyEvent.VK_S) && keyStates.get(KeyEvent.VK_S) ? -speed : 0);
        double leftX = (keyStates.containsKey(KeyEvent.VK_A) && keyStates.get(KeyEvent.VK_A) ? -speed : 0) +
            (keyStates.containsKey(KeyEvent.VK_D) && keyStates.get(KeyEvent.VK_D) ? speed : 0);

        double rightX = (keyStates.containsKey(KeyEvent.VK_LEFT) && keyStates.get(KeyEvent.VK_LEFT) ? -speed : 0) +
            (keyStates.containsKey(KeyEvent.VK_RIGHT) && keyStates.get(KeyEvent.VK_RIGHT) ? speed : 0);
//        Main.field.chassis.updateDrive(leftX, leftY, rightX);
    }

    @Override
    public void keyTyped(KeyEvent e) {
    }

    @Override
    public void keyPressed(KeyEvent e) {
        int key = e.getKeyCode();

        keyStates.put(key, true);
        updateDrive();
    }

    @Override
    public void keyReleased(KeyEvent e) {
        int key = e.getKeyCode();
        if (key == KeyEvent.VK_SPACE) {
            Main.start = true;
        }
        keyStates.put(key, false);
        updateDrive();
    }
}