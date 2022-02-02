// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    // PINOUT Values:

    // DriveTrain
    public static int kLeftFrontDrivePort = 1;
    public static int kRightFrontDrivePort = 3;
    public static int kLeftBackDrivePort = 2;
    public static int kRightBackDrivePort = 4;

    //Shooter

    public static int kLeftShooterPort = 5;
    public static int kRightShooterPort = 7;

    //Sensors:

    public static Port kNavxPort = SerialPort.Port.kMXP;
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.16249;
    public static final double kvVoltSecondsPerMeter = 3.2952;
    public static final double kaVoltSecondsSquaredPerMeter = 0.30496;

    public static final double kPDriveVel = 3.869;

    public static final double kTrackwidthMeters = 0.505;//0.5207
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    // It seems that the meter value is three times what it should be.

    public static final double wheelDiameterInInches = 4;
    public static final double inchesToMetersFactor = 0.0254;
    public static final double shooterGearRatio = 1;
    public static final double gearRatio = 0.12;

    public static final double kEncoderDistancePerPulse = 0.0359;//0.00136

    public static final double rotation = -0.5;
    public static final double speed = 1.0;

}
