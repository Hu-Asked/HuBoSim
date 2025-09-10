public class SimMath {
    private static final double fieldSize = Main.FIELD_SIZE;
    private static final double baseSize = 140;

    public static double inchesToPixels(double inches) {
        return fieldSize/baseSize*inches;
    }

    public static double pixelsToInches(double pixels) {
        return pixels*baseSize/fieldSize;
    }

    public static Structs.Point getLineIntersection(Structs.Line2D line1, Structs.Line2D line2) {
        double startX1 = line1.start.x;
        double startY1 = line1.start.y;
        double endX1 = line1.end.x;
        double endY1 = line1.end.y;
        double startX2 = line2.start.x;
        double startY2 = line2.start.y;
        double endX2 = line2.end.x;
        double endY2 = line2.end.y;

        double slope1 = (endY1 - startY1) / (endX1 - startX1);
        double slope2 = (endY2 - startY2) / (endX2 - startX2);
        if(slope1 == slope2) {
            return null;
        }
        double b1 = startY1 - slope1*startX1;
        double b2 = startY2 - slope2*startX2;

        double ix = (b2-b1)/(slope1-slope2);
        double iy = slope1*ix + b1;

        return new Structs.Point(ix, iy);
    }
}
