import java.util.Random;

public class SimMath {
    private static final double fieldSize = Main.FIELD_SIZE;
    private static final double baseSize = 140;
    public static Random rd = new Random();

    public static double inchesToPixels(double inches) {
        return fieldSize/baseSize*inches;
    }

    public static double pixelsToInches(double pixels) {
        return pixels*baseSize/fieldSize;
    }


    public static Structs.Point getLineIntersection(Structs.Line2D line1, Structs.Line2D line2) {
        double x1 = line1.start.x, y1 = line1.start.y;
        double x2 = line1.end.x, y2 = line1.end.y;
        double x3 = line2.start.x, y3 = line2.start.y;
        double x4 = line2.end.x, y4 = line2.end.y;

        double dn = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (Math.abs(dn) < 1e-8) return null; // Lines are parallel

        double px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / dn;
        double py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / dn;

        // Check if intersection is within both segments
        if (px < Math.min(x1, x2) - 1e-8 || px > Math.max(x1, x2) + 1e-8 ||
                px < Math.min(x3, x4) - 1e-8 || px > Math.max(x3, x4) + 1e-8 ||
                py < Math.min(y1, y2) - 1e-8 || py > Math.max(y1, y2) + 1e-8 ||
                py < Math.min(y3, y4) - 1e-8 || py > Math.max(y3, y4) + 1e-8) {
            return null;
        }

        return new Structs.Point(px, py);
    }

    public static double randomGaussian() {
        return rd.nextGaussian();
    }

}
