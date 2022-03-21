// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these
    // values for your robot.

    // Main Bot Character
    //
    //
    //
    public static final double ksVolts = 0.13271;// 0.16249 0.13271
    public static final double kvVoltSecondsPerMeter = 2.1525;// 3.2952 2.1525
    public static final double kaVoltSecondsSquaredPerMeter = 0.40778;// 0.30496 0.40778

    public static final double kPDriveVel = 2.9285;// 3.869//2.0146

    public static final double kTrackwidthMeters = 0.64;// 0.5207
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

    // Reasonable baseline values for a RAMETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;// 0.5832 2
    public static final double kRamseteZeta = 0.7;// 0.7
    // It seems that the meter value is three times what it should be.

    public static final double wheelDiameterInInches = 6; // TODO: Be more clear on if this is A drive wheel or shooter
                                                          // wheel
    public static final double inchesToMetersFactor = 0.0254; // TODO: remove use Units.inchestoMeter
    public static final double shooterGearRatio = 1;
    public static final double gearRatio = 0.12;

    public static final double motorRotationsPerTurntableRotation = 104.0;


    // public static final double kEncoderDistancePerPulse = 0.0359;//0.00136
    // //TODO: Not used Remove?

    public static final double speedHigh = 0.7;
    public static final double speedLow = 0.35;
    public static final double rotationHigh = -0.60;
    public static final double rotationLow = -0.30;
    public static final double rampRate = 0.2;

    public static final double limelightHeight = 0.69; // based on last year's robot
    public static final double hubHeight = 2.7178;
    public static final double azimuthAngle1 = 40; // mount angle of the limelight
    public static final double metersToFeet = 3.28084;

    public static final double lowPoweredShotRPM = 1200;

    // Drivetrain CAN IDs
    public static final int rightBackMotorID = 1;
    public static final int rightFrontMotorID = 2;
    public static final int leftBackMotorID = 3;
    public static final int leftFrontMotorID = 4;

    // shooter CAN IDs
    public static final int leftShooterMotorID = 5;
    public static final int rightShooterMotorID = 6;

    // climber CAN IDs
    public static final int leftClimberMotorID = 7;
    public static final int rightClimberMotorID = 8;

    // lazysusan CAN IDs
    public static final int lazySusanID = 9;

    // loader CAN IDs
    public static final int intakeMotorID = 10;
    public static final int intakeArmsMotorID = 11;

    // Drivetrain
    public static final IdleMode idleMode = IdleMode.kBrake;
    public static final int currentLimit = 75;
    public static final double deadband = 0.05;

    //climber
    public static final double winchMinLimit = 0;
    public static final double winchMaxLimit = 100;
    public static final int pistonMaxFrames = 15;

    // shooter
    public static final int minShooterRPM = 0;
    public static final int maxShooterRPM = 5500;
    public static final double shooterVibrationTolerance = 0.03;

    // lazysusan
    public static final int turntableThresh = 45;
    public static final double stowedPosition = 50;

    // differential constants (very cool)
    public static final double diffConstLS = 0.014;//0.012 0.014
    public static final double diffConstAutoLS = 0.040;
    public static final double diffConstShooter = 9 * Math.pow(10, -6);
    public static final double diffConstWinchHold = 0.3;
    public static final double diffConstKeepPosition = 0.00001; // TODO: Test this constant and optimize it
    public static final double diffConstTurn = 0.008;

    public static final double diffConstTankDriveAim = 0.014;
    
    public static final int holdTime = 1000;


    public static final double leftBias = 1.3;
    // Sim Vars
    public static final double kSimGearRatio = Constants.gearRatio;
    public static final double kSimTrackwidthMeters = Constants.kTrackwidthMeters;
    public static final double kSimDrivekVLinear = 1.98;
    public static final double ksimDrivekALinear = 0.2;
    public static final double ksimDrivekVAngular = 1.5;
    public static final double kSimDrivekAAngular = 0.3;
    public static final double kSimShooterInertia = 0.5*Units.lbsToKilograms(1.5)*Math.pow(Units.inchesToMeters(4),2); //1/2*M*R^2 
    public static final double kSimTurntableInertia = 0.5*Units.lbsToKilograms(25)*Math.pow(Units.inchesToMeters(6),2); //TODO: switch to ring formula vs disk
    public static final double kSimTurntableGearRatio = 104.0;//5*(156.0/16.0);
    public static final double kSimUpdateTime = 0.02;
    public static final double kSimRobotWeight = Units.lbsToKilograms(120);
    public static final double kSimClimberGearRatio = 1;
    public static final double kSimClimberDrumSize = 0.2;
    public static final double ksimClimberMaxHeight = Units.feetToMeters(6);
}
