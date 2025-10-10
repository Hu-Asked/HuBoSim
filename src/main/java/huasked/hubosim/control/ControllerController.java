package huasked.hubosim.control;

import huasked.hubosim.Main;
import huasked.hubosim.SimMath;

import java.awt.event.KeyEvent;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

import static org.lwjgl.glfw.GLFW.*;

public class ControllerController {
    public double leftStickX = 0;
    public double leftStickY = 0;
    public double rightStickX = 0;
    public double rightStickY = 0;
    int jid = GLFW_JOYSTICK_1; // Default to first joystick

    public ControllerController() {
    }

    public void init() {
        if (!glfwInit()) {
            System.out.println("Failed to initialize GLFW");
            return;
        }
        int joystickCount = 0;
        for (int i = GLFW_JOYSTICK_1; i <= GLFW_JOYSTICK_LAST; i++) {
            if (glfwJoystickPresent(i)) {
                System.out.println("Found joystick: " + i + " - " + glfwGetJoystickName(i));
                joystickCount++;
            }
        }

        if (joystickCount == 0) {
            System.err.println("No controllers detected!");
            return;
        }
    }
    public void pollController() {
        if (!glfwJoystickPresent(jid)) return;

        // Get button states
        ByteBuffer buttons = glfwGetJoystickButtons(jid);
        int buttonsCount = buttons.limit();

        // Get axis states (triggers/thumbsticks)
        FloatBuffer axes = glfwGetJoystickAxes(jid);
        int axesCount = axes.limit();
        // PS4 Axis Mappingq
        if (axesCount >= 6) {
            this.leftStickX = axes.get(0);   // Left Stick X
            this.leftStickY = -axes.get(1);   // Left Stick Y
            this.rightStickX = axes.get(2);  // Right Stick X
            this.rightStickY = -axes.get(5);  // Right Stick Y
        }
    }

}