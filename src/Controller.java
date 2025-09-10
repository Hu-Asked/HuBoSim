import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.HashMap;

public class Controller implements KeyListener {
    // Track key states
    private HashMap<Integer, Boolean> keyStates = new HashMap<>();

    // Update drive based on current key states
    private void updateDrive() {
        double forward = 0;
        double turn = 0;

        // Calculate forward component
        if (keyStates.containsKey(KeyEvent.VK_W) && keyStates.get(KeyEvent.VK_W)) forward += 1.0;
        if (keyStates.containsKey(KeyEvent.VK_S) && keyStates.get(KeyEvent.VK_S)) forward -= 1.0;

        // Calculate turn component
        if (keyStates.containsKey(KeyEvent.VK_A) && keyStates.get(KeyEvent.VK_A)) turn += 1.0;
        if (keyStates.containsKey(KeyEvent.VK_D) && keyStates.get(KeyEvent.VK_D)) turn -= 1.0;

        // Apply arcade drive formula
        double leftPower = forward + turn;
        double rightPower = forward - turn;

        // Ensure powers are within valid range [-1.0, 1.0]
        leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
        rightPower = Math.max(-1.0, Math.min(1.0, rightPower));
        Main.field.chassis.leftDrive(leftPower);
        Main.field.chassis.rightDrive(rightPower);
    }

    @Override
    public void keyTyped(KeyEvent e) {
        return;
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