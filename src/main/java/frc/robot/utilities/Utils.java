package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;

public class Utils {
    public static double map(final double value, final double input_min, final double input_max,
            final double output_min, final double output_max) {
        final double scaler = (output_max - output_min) / (input_max - input_min);
        final double value_scaled = ((value - input_min) * scaler) + output_min;

        return value_scaled;
    }

    public static double magnitude(final double x, final double y) {
        return Math.pow(Math.pow(x, 2) + Math.pow(y, 2), 0.5);
    }

    /**
     * Convert Cartisan coords into angle from 0 to 2PI
     * 
     * @param x
     * @param y
     * @return an angle from 0 to 2PI
     */
    public static double angle(final double x, final double y) {
        return normalizeAngle(Math.atan2(y, x)); // returns 0 to 2PI
    }

    /**
     * A function that adds two polar vectors
     * 
     * @param v1 a vector in the form [theta, magnitude]
     * @param v2 another vector in the form [theta, magnitude]
     * @return the vector sum, in the form [theta, magnitude]
     */
    public static double[] addVectors(double[] v1, double[] v2) {
        // Seperate the vectors into variables
        double vector1Angle = v1[0];
        double vector1Mag = v1[1];
        double vector2Angle = v2[0];
        double vector2Mag = v2[1];

        // Then seperate into their components
        double vector1x = vector1Mag * Math.cos(vector1Angle);
        double vector1y = vector1Mag * Math.sin(vector1Angle);
        double vector2x = vector2Mag * Math.cos(vector2Angle);
        double vector2y = vector2Mag * Math.sin(vector2Angle);

        // Add the components
        double vectorRx = vector1x + vector2x;
        double vectorRy = vector1y + vector2y;

        // Turn it back into a polar vector
        double vectorRAngle = Utils.angle(vectorRx, vectorRy);
        double vectorRMag = Utils.magnitude(vectorRx, vectorRy);

        return new double[] {vectorRAngle, vectorRMag};
    }

    /**
     * A function that normalizes an angle to the range [0, 2PI)
     * 
     * @param angle angle to be normalized
     * @return angle in the range [0, 2PI)
     */
    public static double normalizeAngle(double angle) {
        angle %= 2 * Math.PI;
        if (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * 
     */
    public static <X extends Comparable<X>> X max(X[] list) {
        X max = list[0];

        for (X x : list)
            if (x.compareTo(max) > 0)
                max = x;

        return max;
    }

    /**
     * Returns value limited to the range [lowerLimit, upperLimit]
     * 
     * @param val
     * @param lowerLimit
     * @param upperLimit
     * @return value limited to the range [lowerLimit, upperLimit]
     */
    public static double limitRange(double val, double lowerLimit, double upperLimit) {
        if (val > upperLimit)
            return upperLimit;
        if (val < lowerLimit)
            return lowerLimit;
        return val;
    }

    public static boolean almostEqual(Pose2d a, Pose2d b, double epsilon) {
        return Math.abs(a.getX() - b.getY()) + Math.abs(a.getY() - b.getY())
                + Math.abs(a.getRotation().getRadians() - b.getRotation().getRadians()) < epsilon;
    }
}
