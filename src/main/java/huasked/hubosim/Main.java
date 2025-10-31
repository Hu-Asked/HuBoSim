package huasked.hubosim;

import javax.swing.JFrame;

import huasked.hubosim.algorithm.PurePursuit;
import huasked.hubosim.chassis.TankDrive;
import huasked.hubosim.control.KeyboardController;

public class Main {
    public static boolean start = false;
    public static double[] expectedDist = {-1, -1, -1, -1};
    public static JFrame frame = new JFrame("HuBoSim");
    public static KeyboardController keyboard = new KeyboardController();
    public static PurePursuit pp;
    // public static ControllerController controller = new ControllerController();

    public static void main(String[] args) {
        Field field = new Field(140, 140, 3);
        /*   SWERVE CONFIGURATION
        SwerveModule l1, l2, r1, r2;
        l1 = new SwerveModule(2, 2);
        l2 = new SwerveModule(2, 2);
        r1 = new SwerveModule(2, 2);
        r2 = new SwerveModule(2, 2);
        SwerveDrive chassis = new SwerveDrive(12, 12, 3.5, new Pose(0, 0, 0));
        SwerveModule[] modules = {l1, l2, r1, r2};
        chassis.addSwerveModules(l1, r1, l2, r2);
        SwerveDrive chassis = new SwerveDrive(0, 0, 10, 12, modules);
        */
        PPPaths.init();
        PPPaths.setActive("elims");
        TankDrive chassis = new TankDrive(0, 0, 10, 12, 1.5);
        pp = new PurePursuit(chassis, 1, 0.002, 10);
        chassis.pose.x = PPPaths.activePath.get(0).point.x;
        chassis.pose.y = PPPaths.activePath.get(0).point.y;
        pp.currentPose = chassis.pose;
        pp.initializePath(pp.getStrippedPath(PPPaths.activePath));
        field.addElement(chassis);
        setFrame(field);
        // ControllerController controller = new ControllerController();
        // controller.init();
        try {
            while (!start) {
                Thread.sleep(1000);
            }
        }
        catch (InterruptedException e) {
            return;
        }
        while (true) {
            // controller.pollController();
            // chassis.updateDrive(controller.leftStickX, controller.leftStickY, controller.rightStickX, controller.rightStickY);
            chassis.updateDrive(keyboard.leftX, keyboard.leftY);
            pp.followPath(PPPaths.activePath, 10, 1, 1);
            // for (Lidar.Direction dir : Lidar.Direction.values()) {
            //     int i = dir.ordinal();
            //     expectedDist[i] = mcl.getParticleReading(chassis.pose, dir);
            // }
            // lidar.updateSensorLines();
            // mcl.update();
            pp.currentPose = chassis.pose;
            field.update();
            tick();
        }
    }

    static void setFrame(Field field) {
        frame.add(field);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.addKeyListener(keyboard);
        frame.pack();
        frame.setFocusable(true);
        frame.requestFocusInWindow();
        frame.setVisible(true);
        frame.setResizable(true);
    }
    static void tick() {
            try {
                Thread.sleep(16);
            }
            catch (InterruptedException e) {
                return;
            }
    }
}
