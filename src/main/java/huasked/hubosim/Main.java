package huasked.hubosim;

import huasked.hubosim.control.ControllerController;
import huasked.hubosim.control.KeyboardController;
import huasked.hubosim.util.Point;
import huasked.hubosim.util.Pose;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.Map;

public class Main {
    public static ArrayList<Map.Entry<Point, Double>> chosenPath = PPPaths.samplePath;
    public static boolean start = false;
    public static double[] expectedDist = {-1, -1, -1, -1};
    public static JFrame frame = new JFrame("HuBoSim");
    public static KeyboardController master = new KeyboardController();
//    public static ControllerController controller = new ControllerController();

    public static void main(String[] args) {
        SwerveModule l1, l2, r1, r2;
        l1 = new SwerveModule(2, 2);
        l2 = new SwerveModule(2, 2);
        r1 = new SwerveModule(2, 2);
        r2 = new SwerveModule(2, 2);
//        Chassis chassis = new Chassis(12, 12, 3.5, new Pose(0, 0, 0));
//        chassis.addSwerveModules(l1, r1, l2, r2);
//        PurePursuit pp = new PurePursuit(chassis, 1, 0.002, 10);
//        chassis.pose.x = chosenPath.getFirst().getKey().x;
//        chassis.pose.y = chosenPath.getFirst().getKey().y;
//        pp.currentPose = chassis.pose;
//        pp.initializePath(pp.getStrippedPath(chosenPath));
        Field field = new Field(140, 140, 3);
        Chassis chassis = new Chassis(0, 0, 10, 12);
        chassis.addSwerveModules(l1, l2, r1, r2);
        ControllerController controller = new ControllerController();
        field.addElement(chassis);
        setFrame(field);
        controller.init();
        try {
            while (!start) {
                Thread.sleep(1000);
            }
        }
        catch (InterruptedException e) {
            return;
        }
        while (true) {
            controller.pollController();
            chassis.updateDrive(controller.leftStickX, controller.leftStickY, controller.rightStickX, controller.rightStickY);
//            for (Lidar.Direction dir : Lidar.Direction.values()) {
//                int i = dir.ordinal();
//                expectedDist[i] = mcl.getParticleReading(chassis.pose, dir);
//            }
//            lidar.updateSensorLines();
//            mcl.update();
//            chassis.updateDrive(controller.leftStickX, controller.leftStickY, controller.rightStickX, controller.rightStickY);
//            pp.currentPose = chassis.pose;
//            if(!pp.exit) pp.followPath(chosenPath, 10, 1, 4);
//            pp.waitUntil(60);
//            if(pp.release) {
//                chosenPath = PPPaths.samplePath1;
//                pp.initializePath(pp.getStrippedPath(chosenPath));
//            }
            field.update();
            try {
                Thread.sleep(16);
            }
            catch (InterruptedException e) {
                return;
            }
        }
    }

    static void setFrame(Field field) {
        frame.add(field);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.pack();
        frame.setFocusable(true);
        frame.addKeyListener(master);
        frame.requestFocusInWindow();
        frame.setVisible(true);
        frame.setResizable(true);
    }
}
