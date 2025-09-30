import util.VelocityVector;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.HashMap;

public class Controller implements KeyListener {
    // Track key states
    private HashMap<Integer, Boolean> keyStates = new HashMap<>();

    // Update drive based on current key states
    private void updateDrive() {
        int leftY = (keyStates.containsKey(KeyEvent.VK_W) && keyStates.get(KeyEvent.VK_W) ? 1 : 0) +
                    (keyStates.containsKey(KeyEvent.VK_S) && keyStates.get(KeyEvent.VK_S) ? -1 : 0);
        int leftX = (keyStates.containsKey(KeyEvent.VK_A) && keyStates.get(KeyEvent.VK_A) ? -1 : 0) +
                    (keyStates.containsKey(KeyEvent.VK_D) && keyStates.get(KeyEvent.VK_D) ? 1 : 0);

        int rightX = (keyStates.containsKey(KeyEvent.VK_LEFT) && keyStates.get(KeyEvent.VK_LEFT) ? -1 : 0) +
                     (keyStates.containsKey(KeyEvent.VK_RIGHT) && keyStates.get(KeyEvent.VK_RIGHT) ? 1 : 0);

        double L = SimMath.pixelsToInches(Main.field.chassis.length);
        double W = SimMath.pixelsToInches(Main.field.chassis.width);
        double R = Math.hypot(L, W);

        double a = leftX - rightX * (L / R);
        double b = leftX + rightX * (L / R);
        double c = leftY - rightX * (W / R);
        double d = leftY + rightX * (W / R);

        double backRightSpeed = Math.hypot(a, d);
        double backLeftSpeed = Math.hypot(a, c);
        double frontRightSpeed = Math.hypot(b, d);
        double frontLeftSpeed = Math.hypot(b, c);

        double backRightAngle = Math.atan2(a, c);
        double backLeftAngle = Math.atan2(a, d);
        double frontRightAngle = Math.atan2(b, c);
        double frontLeftAngle = Math.atan2(b, d);

        Main.field.chassis.modules[0].setAngleRads(frontLeftAngle);
        Main.field.chassis.modules[0].setSpeed(frontLeftSpeed);
        Main.field.chassis.modules[1].setAngleRads(frontRightAngle);
        Main.field.chassis.modules[1].setSpeed(frontRightSpeed);
        Main.field.chassis.modules[2].setAngleRads(backLeftAngle);
        Main.field.chassis.modules[2].setSpeed(backLeftSpeed);
        Main.field.chassis.modules[3].setAngleRads(backRightAngle);
        Main.field.chassis.modules[3].setSpeed(backRightSpeed);
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
        if(key == KeyEvent.VK_SPACE) {
            Main.start = true;
        }
        keyStates.put(key, false);
        updateDrive();
    }
}