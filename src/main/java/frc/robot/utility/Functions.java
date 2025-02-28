package frc.robot.utility;

public class Functions {

    /** Finds the smallest difference between two angles or gets the equivalent angle between -180 and
     * 180 when the currentAngle is 0 (and wrapAngle is 360).
     * A wrapAngle of 180 treats the targetAngle and the angle directly opposite of targetAngle the same
     *
     * @param currentAngle in degrees
     * @param targetAngle in degrees
     * @param wrapAngle Should be 360 unless you are doing a swerve module
     */
    public static double angleDifferenceDeg(double currentAngle, double targetAngle, int wrapAngle) {
        double result1 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), wrapAngle * 100L) * 0.01;
        double result2 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), -wrapAngle * 100L) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;
    }
    /** Finds the smallest difference between two angles or gets the equivalent angle between -pi and
     * pi when the currentAngle is 0 (and wrapAngle is 2pi).
     * A wrapAngle of pi treats the targetAngle and the angle directly opposite of targetAngle the same
     *
     * @param currentAngle in radians
     * @param targetAngle in radians
     * @param wrapAngle Should be 2pi unless you are doing a swerve module
     */
    public static double angleDifference(double currentAngle, double targetAngle, double wrapAngle) {
        return Math.toRadians(angleDifferenceDeg(Math.toDegrees(currentAngle), Math.toDegrees(targetAngle), (int) Math.round(Math.toDegrees(wrapAngle))));
    }
    /**
     * Finds the equivalent angle between -180 and 180
     * 
     * @param angle in degrees
     */
    public static double normalizeAngleDeg(double angle) {
        return angleDifferenceDeg(0, angle, 360);
    }
    /**
     * Finds the equivalent angle between -pi and pi
     * 
     * @param angle in radians
     */
    public static double normalizeAngle(double angle) {
        return angleDifference(0, angle, 2*Math.PI);
    }
    /**
     * Finds the equivalent angle between 0 and 360
     * 
     * @param angle in degrees
     */
    public static double normalizeAngleDegPositive(double angle) {
        return angleDifferenceDeg(-180, angle, 360) + 180;
    }
    /**
     * Finds the equivalent angle between 0 and 2pi
     * 
     * @param angle in radians
     */
    public static double normalizeAnglePositive(double angle) {
        return angleDifference(-2*Math.PI, angle, 2*Math.PI) + 2*Math.PI;
    }
    /**
     * Works the same way as the round function in python. 
     * @param input double
     * @param decimalPlaces Number of decimal places to round to
     */
    public static double round(double input, int decimalPlaces) {
        return Math.round(input * (Math.pow(10, decimalPlaces))) / (Math.pow(10, decimalPlaces));
    }

    /**
     * Limits a value to between the min and max. 
     */
    public static double minMaxValue(double min, double max, double value) {
        if (value > max) return max;
        else return Math.max(value, min);
    }

    /**
     * Applies a polynomial curve to the input value while keeping the same sign. 
     * @param value input
     * @param mag power of the curve
     */
    public static double throttleCurve(double value, double mag) {
        return Math.abs(Math.pow(value, mag)) * Math.signum(value); 
    }

    /**
     * This is because applying a throttle curve to each X and Y individually creates a bias towards moving orthogonally
     * A curve mag of 1 would just apply the x and y normally, 2 would apply a quadratic curve, etc.
     */
    public static double getXOfVectorMag(double X, double Y, double throttleCurveMag) {
        return throttleCurve(Math.hypot(X,Y), throttleCurveMag) * Math.cos(Math.atan2(Y,X));
    }

    /**
     * This is because applying a throttle curve to each X and Y individually creates a bias towards moving orthogonally
     * A curve mag of 1 would just apply the x and y normally, 2 would apply a quadratic curve, etc.
     */
    public static double getYOfVectorMag(double X, double Y, double throttleCurveMag) {
        return throttleCurve(Math.hypot(X,Y), throttleCurveMag) * Math.sin(Math.atan2(Y,X));
    }

    public static double getXOfVectorWithMag(double X, double Y, double mag, double throttleCurveMag) {
        return throttleCurve(mag, throttleCurveMag) * Math.cos(Math.atan2(Y,X));
    }

    public static double getYOfVectorWithMag(double X, double Y, double mag, double throttleCurveMag) {
        return throttleCurve(mag, throttleCurveMag) * Math.sin(Math.atan2(Y,X));
    }


}
