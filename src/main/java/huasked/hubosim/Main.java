package huasked.hubosim;

import javax.swing.JFrame;

import huasked.hubosim.algorithm.Boomerang;
import huasked.hubosim.algorithm.PurePursuit;
import huasked.hubosim.chassis.TankDrive;
import huasked.hubosim.control.KeyboardController;
import huasked.hubosim.control.MouseController;

public class Main {
    public static boolean start = false;
    public static double[] expectedDist = {-1, -1, -1, -1};
    public static JFrame frame = new JFrame("HuBoSim");
    public static KeyboardController keyboard = new KeyboardController();
    public static MouseController mouse = new MouseController();
    public static PurePursuit pp;
    public static Field field;
    public static Boomerang boom;
    public static TankDrive chassis;
    // public static ControllerController controller = new ControllerController();

    public static void main(String[] args) {
        field = new Field(140, 140, 3);
        FieldElements elements = new FieldElements();
        /*   SWERVE CONFIGURATION */
        // SwerveModule l1, l2, r1, r2;
        // l1 = new SwerveModule(2, 2);
        // l2 = new SwerveModule(2, 2);
        // r1 = new SwerveModule(2, 2);
        // r2 = new SwerveModule(2, 2);
        // SwerveModule[] modules = {l1, l2, r1, r2};
        // SwerveDrive chassis = new SwerveDrive(0, 0, 10, 12, modules);

        PPPaths.init();
        PPPaths.setActive("elims");
        /*TankDrive*/ chassis = new TankDrive(0, 0, 10, 12, 1.5);
        /*Boomerang */boom = new Boomerang(0.05, 0.05, 10, chassis);
        // pp = new PurePursuit(chassis, 1, 0.002, 10);
        // pp.currentPose = chassis.pose;
        // pp.initializePath(pp.getStrippedPath(PPPaths.activePath));
        chassis.pose.x = PPPaths.activePath.get(0).point.x;
        chassis.pose.y = PPPaths.activePath.get(0).point.y;
        field.addElement(chassis);
        field.addElement(mouse);
        field.addElement(elements);
        field.addElement(boom);
        setFrame(field);
        //         ControllerController controller = new ControllerController();
        // controller.init();
        try {
            Thread.sleep(1500);
        }
        catch (InterruptedException e) {
            return;
        }
        System.out.println("started");
        while (true) {
            // controller.pollController();
            // chassis.updateDrive(controller.leftStickX, controller.leftStickY, controller.rightStickX, controller.rightStickY);
            // chassis.updateDrive(keyboard.leftX, keyboard.leftY, keyboard.rightX, 0);
            chassis.updateDrive(keyboard.leftX, keyboard.leftY);
            boom.pose = chassis.pose;
            // pp.followPath(PPPaths.activePath, 14, 0.8, 1);
            // pp.currentPose = chassis.pose;
            // for (Lidar.Direction dir : Lidar.Direction.values()) {
            //     int i = dir.ordinal();
            //     expectedDist[i] = mcl.getParticleReading(chassis.pose, dir);
            // }
            // lidar.updateSensorLines();
            // mcl.update();
            //
            if(mouse.newPress) {
                mouse.pose.heading = (chassis.pose.x > mouse.pose.x) ? 3 * Math.PI/2 : Math.PI/2;
                boom.moveTo(mouse.pose, 0.7, 5); 
                mouse.newPress = false;
            }
            tick();
        }
    }

    static void setFrame(Field field) {
        frame.add(field);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.addKeyListener(keyboard);
        frame.addMouseListener(mouse);
        frame.pack();
        frame.setFocusable(true);
        frame.requestFocusInWindow();
        frame.setVisible(true);
        frame.setResizable(true);
    }
    public static void tick() {
        field.update();
        boom.pose = chassis.pose;
        try {
            Thread.sleep(16);
        }
        catch (InterruptedException e) {
            return;
        }
    }
}
