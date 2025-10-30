package huasked.hubosim.control;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.HashMap;

import huasked.hubosim.Main;

public class KeyboardController implements KeyListener {
    private HashMap<Integer, Boolean> keyStates = new HashMap<>();
    public double leftY = 0;
    public double leftX = 0;
    public double rightX = 0;
    public KeyboardController() {}
    double speed = 1;

    private void updateDrive() {
        leftY = (keyStates.containsKey(KeyEvent.VK_W) && keyStates.get(KeyEvent.VK_W) ? speed : 0) +
            (keyStates.containsKey(KeyEvent.VK_S) && keyStates.get(KeyEvent.VK_S) ? -speed : 0);
        leftX = (keyStates.containsKey(KeyEvent.VK_A) && keyStates.get(KeyEvent.VK_A) ? -speed : 0) +
            (keyStates.containsKey(KeyEvent.VK_D) && keyStates.get(KeyEvent.VK_D) ? speed : 0);

        rightX = (keyStates.containsKey(KeyEvent.VK_LEFT) && keyStates.get(KeyEvent.VK_LEFT) ? -speed : 0) +
            (keyStates.containsKey(KeyEvent.VK_RIGHT) && keyStates.get(KeyEvent.VK_RIGHT) ? speed : 0);
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
