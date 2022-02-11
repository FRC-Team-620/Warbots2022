// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.15405;//0.16249
    public static final double kvVoltSecondsPerMeter = 3.3214;//3.2952
    public static final double kaVoltSecondsSquaredPerMeter = 0.33697;//0.30496

    public static final double kPDriveVel = 3.9997;//3.869//2.0146

    public static final double kTrackwidthMeters = 0.505;//0.5207
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.2;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;//0.5832 2
    public static final double kRamseteZeta = 0.0125;//0.7
    // It seems that the meter value is three times what it should be.

    public static final double wheelDiameterInInches = 4;
    public static final double inchesToMetersFactor = 0.0254;
    public static final double shooterGearRatio = 1;
    public static final double gearRatio = 0.12;

    public static final double kEncoderDistancePerPulse = 0.0359;//0.00136 

    public static final double speed = 1.0;
    public static final double rotation = -0.5;
    public static final double rampRate = 0.0;

    public static final double limelightHeight = 0.5588; //based on last year's robot
    public static final double hubHeight = 2.7178;
    public static final double azimuthAngle1 = 37.8473967029; // mount angle of the limelight
    public static final double metersToFeet = 3.28084;

    public static final Map<String, List<Double>> driverSpecificOptions = new HashMap() {{
        //List(double speed, double rotation, double rampRate)
        put("default", Arrays.asList(speed, rotation, rampRate));
    }};
}
