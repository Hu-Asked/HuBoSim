import util.Pose;
import util.VelocityVector;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.Map;

public class Main {
    static Field field;
    static Lidar lidar;
    static MCL mcl;
    public static final int FIELD_SIZE = 880;
    public static ArrayList<Map.Entry<util.Point, Double>> chosenPath = PPPaths.samplePath;
    public static boolean start = false;
    public static double[] expectedDist = {-1, -1, -1, -1};

    public static void main(String[] args) {
        JFrame frame = new JFrame("HuBoSim");
        SwerveModule l1, l2, r1, r2;
        l1 = new SwerveModule(2, 2);
        l2 = new SwerveModule(2, 2);
        r1 = new SwerveModule(2, 2);
        r2 = new SwerveModule(2, 2);
        Chassis chassis = new Chassis(12, 12, 3.5, new Pose(0, 0, 0));
        chassis.addSwerveModules(l1, r1, l2, r2);

        Controller master = new Controller();
        PurePursuit pp = new PurePursuit(chassis, 1, 0.002, 10);
        field = new Field(chassis, pp);
        lidar = new Lidar(50, chassis);
        mcl = new MCL(0, 0.01, -3, -3, lidar, chassis);
        field.setPreferredSize(new Dimension((int) field.WIDTH, (int) field.HEIGHT));
        frame.add(field);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.pack();
        frame.setFocusable(true);
        frame.addKeyListener(master);
        frame.requestFocusInWindow();
        frame.setVisible(true);
        frame.setResizable(false);
//        chassis.pose.x = chosenPath.getFirst().getKey().x;
//        chassis.pose.y = chosenPath.getFirst().getKey().y;
//        pp.currentPose = chassis.pose;
//        pp.initializePath(pp.getStrippedPath(chosenPath));
        try {
            while(!start) {
                Thread.sleep(1000);
            }
        } catch (InterruptedException e) {
            return;
        }
        while(true) {
//            for (Lidar.Direction dir : Lidar.Direction.values()) {
//                int i = dir.ordinal();
//                expectedDist[i] = mcl.getParticleReading(chassis.pose, dir);
//            }
//            lidar.updateSensorLines();
//            mcl.update();
            pp.currentPose = chassis.pose;
//            if(!pp.exit) pp.followPath(chosenPath, 10, 1, 4);
//            pp.waitUntil(60);
//            if(pp.release) {
//                chosenPath = PPPaths.samplePath1;
//                pp.initializePath(pp.getStrippedPath(chosenPath));
//            }

            field.updateField();
            try {
                Thread.sleep(16);
            } catch (InterruptedException e) {
                return;
            }
        }
    }
}
