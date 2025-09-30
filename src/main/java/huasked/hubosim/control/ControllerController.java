package huasked.hubosim.control;

import net.java.games.input.*;

public class ControllerController {
    public ControllerController() {
        Event event = new Event();

        Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();
        for (int i = 0; i < controllers.length; i++) {
            controllers[i].poll();

            EventQueue queue = controllers[i].getEventQueue();

            while (queue.getNextEvent(event)) {
                Component comp = event.getComponent();

                comp.getPollData();
            }
        }
    }
}
