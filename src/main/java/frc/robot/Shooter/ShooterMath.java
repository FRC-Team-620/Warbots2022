package frc.robot.Shooter;

import frc.robot.Constants;

public final class ShooterMath {
    public static double getDistanceInMeters(double a1, double a2, double h1, double h2) {
        return (h2 - h1) / Math.tan(Math.toRadians(a1 + a2));
    }

    public static double metersToRPM(double meters) {
        double distanceInFeet = Constants.metersToFeet * meters;
        return 117.3708 * distanceInFeet + 1700;
    }

    public static long roundUpToNearestMultiple(double input, int step) {
        int i = (int) Math.ceil(input);
        return i + step - 1;
    }

    // Takes in a boolean of which direction the encoder will be moving in
    // and the current position of the encoder, to see whether it is in the boundaries.
    public static boolean inBounds(boolean directionIsPositive, double encoderPosition) {
        return !((directionIsPositive && encoderPosition >= Constants.turntableThresh) 
        || (!directionIsPositive && encoderPosition <= -Constants.turntableThresh));
    }
}