// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.text.DecimalFormat;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class Drivetrain extends SubsystemBase {
  protected final CANSparkMax leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
  protected final DifferentialDrive diffDrive;
  protected final RelativeEncoder leftBackEncoder, leftFrontEncoder, rightBackEncoder, rightFrontEncoder;
  protected final double countsPerMotorRevolution;
  protected final DecimalFormat decFormat = new DecimalFormat("#.#");
  // The gyro sensor
  protected final Gyro gyro;
  
  // Odometry class for tracking robot pose
  protected final DifferentialDriveOdometry odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFrontMotor = new CANSparkMax(Constants.kLeftFrontDrivePort, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(Constants.kRightFrontDrivePort, MotorType.kBrushless);
    leftBackMotor = new CANSparkMax(Constants.kLeftBackDrivePort, MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(Constants.kRightBackDrivePort, MotorType.kBrushless);

    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();

    IdleMode mode = IdleMode.kBrake; //brakes
    leftFrontMotor.setIdleMode(mode);
    leftBackMotor.setIdleMode(mode);
    rightFrontMotor.setIdleMode(mode);
    rightBackMotor.setIdleMode(mode);

    double openLoopRampRate = 0.6; //0.6 sec to full velocity
    leftFrontMotor.setOpenLoopRampRate(openLoopRampRate);
    leftBackMotor.setOpenLoopRampRate(openLoopRampRate);
    rightFrontMotor.setOpenLoopRampRate(openLoopRampRate);
    rightBackMotor.setOpenLoopRampRate(openLoopRampRate);

    int currentLimit = 45; //maxmium amps
    leftFrontMotor.setSmartCurrentLimit(currentLimit);
    leftBackMotor.setSmartCurrentLimit(currentLimit);
    rightFrontMotor.setSmartCurrentLimit(currentLimit);
    rightBackMotor.setSmartCurrentLimit(currentLimit);

    leftFrontMotor.follow(leftBackMotor, false); //false means not inverted, and true means inverted
    rightFrontMotor.follow(rightBackMotor, false);

    // Encoder creation
    leftBackEncoder = leftBackMotor.getEncoder();
    leftFrontEncoder = leftFrontMotor.getEncoder();
    rightBackEncoder = rightBackMotor.getEncoder();
    rightFrontEncoder = rightFrontMotor.getEncoder();

    countsPerMotorRevolution = leftBackEncoder.getCountsPerRevolution(); 
    //this choice of encoder is arbitrary -- any other encoder would work just as well

    // var conversionFactor = Constants.gearRatio * Constants.wheelDiameterInInches * Constants.inchesToMetersFactor * Math.PI;
    // leftBackEncoder.setPositionConversionFactor(conversionFactor);
    // leftFrontEncoder.setPositionConversionFactor(conversionFactor);
    // rightFrontEncoder.setPositionConversionFactor(conversionFactor);
    // rightBackEncoder.setPositionConversionFactor(conversionFactor);

    gyro = new AHRS(SerialPort.Port.kMXP);

    diffDrive = new DifferentialDrive(leftBackMotor, rightBackMotor);
    diffDrive.setDeadband(0.05); //minmal signal
    rightBackMotor.setInverted(true);
    resetEncoders();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getDistance(leftBackEncoder), getDistance(rightBackEncoder));
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftBackMotor.setVoltage(leftVolts);
    rightBackMotor.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public CANSparkMax getMotor(int idx) {
    idx = (idx-1)%4+1;
    switch(idx) {
      case 1:
        return leftFrontMotor;
      case 2:
        return leftBackMotor;
      case 3:
        return rightFrontMotor;
    }
    return rightBackMotor;
  }
  public RelativeEncoder getEncoder(int idx) {
    idx = (idx-1)%4+1;
    switch(idx) {
      case 1:
        return leftFrontEncoder;
      case 2:
        return leftBackEncoder;
      case 3:
        return rightFrontEncoder;
    }
    return rightBackEncoder;
  }

  public void curvatureInput(double speed, double rotation, boolean isCurvatureDrive) {
    diffDrive.curvatureDrive(speed, rotation, isCurvatureDrive);
  }

  public void leftBackMotorDrive(double x) {
    leftBackMotor.set(x);
  }

  public void rightBackMotorDrive(double x) {
    rightBackMotor.set(x);
  }

  public void leftFrontMotorDrive(double x) {
    leftFrontMotor.set(x);
  }

  public void rightFrontMotorDrive(double x) {
    rightFrontMotor.set(x);
  }

  public double getRPM(int idx) {
    idx = (idx-1)%4+1;
    return getEncoder(idx).getVelocity() * Constants.gearRatio;
  }
  
  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getDistance(leftBackEncoder), getDistance(rightBackEncoder));
  }

  protected double getDistance(RelativeEncoder enc) {
    return enc.getPosition() * enc.getPositionConversionFactor();
  }
}
