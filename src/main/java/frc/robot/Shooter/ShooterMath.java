package frc.robot.Shooter;

import frc.robot.Constants;

public abstract class ShooterMath
{

    public static double getDistanceInMeters(double a1, double a2, double h1, double h2) {
        return (h2 - h1) / Math.tan((a1 + a2) * (Constants.degreesToRadians));
    }

    public static double metersToRPM(double meters) {
        double distanceInFeet = Constants.metersToFeet * meters;
        // System.out.println("Distance In Meters " + meters);
        //return 117.3708 * distanceInFeet + 1632.61502;
        return 117.3708 * distanceInFeet + 1700;
    }

    public static long roundUpToNearestMultiple(double input, int step) {
        int i = (int) Math.ceil(input);
        return ((i + step - 1) / step) * step;
    }
}