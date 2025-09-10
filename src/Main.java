import Structs.Point;
import Structs.Pose;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.Map;
import java.util.AbstractMap.SimpleEntry;

public class Main {
    static Field field;
    static Lidar lidar;
    public static final int FIELD_SIZE = 880;
    public static ArrayList<Map.Entry<Structs.Point, Double>> chosenPath = PPPaths.simplePath;
    public static boolean start = false;

    public static void main(String[] args) {
        JFrame frame = new JFrame("HuBoSim");
        Chassis chassis = new Chassis(10, 12, 3.5, new double[]{0, 0, 0});
        Controller master = new Controller();
        PurePursuit pp = new PurePursuit(chassis, 1, 0.002, 10);
        field = new Field(chassis, pp);
        lidar = new Lidar(50, chassis);
        field.setPreferredSize(new Dimension((int) field.WIDTH, (int) field.HEIGHT));
        frame.add(field);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.pack();
        frame.setFocusable(true);
        frame.addKeyListener(master);
        frame.requestFocusInWindow();
        frame.setVisible(true);
        frame.setResizable(false);
        chassis.pose[0] = chosenPath.getFirst().getKey().x;
        chassis.pose[1] = -chosenPath.getFirst().getKey().y;
        pp.currentPose = new Pose(chassis.pose[0], -chassis.pose[1], chassis.pose[2]);
        pp.initializePath(pp.getStrippedPath(chosenPath));
        try {
            while(!start) {
                Thread.sleep(1000);
            }
        } catch (InterruptedException e) {
            return;
        }
        while(true) {
            lidar.updateSensorLines();
            pp.currentPose = new Pose(chassis.pose[0], -chassis.pose[1], chassis.pose[2]);
            if(!pp.exit) pp.followPath(chosenPath, 10, 1, 2);
            field.updateField(); // approx 60 FPS
            try {
                Thread.sleep(16);
            } catch (InterruptedException e) {
                return;
            }
        }
    }
}