// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  // motor controllers and drive subsystem
  private CANSparkMax leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
  private DifferentialDrive diffDrive;

  // sensors
  private RelativeEncoder leftBackEncoder, leftFrontEncoder, rightBackEncoder, rightFrontEncoder;
  private AHRS navx;

  // Odometry class for tracking robot pose
  private DifferentialDriveOdometry odometry;

  // Constants (put in separate file later)
  private double countsPerMotorRevolution;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // initialize motors
    initMotors();
    initSensors();
  }

  /* INITIALIZATION */
  // motors
  private void initMotors() {
    //initialize
    leftFrontMotor = new CANSparkMax(1, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(3, MotorType.kBrushless);
    leftBackMotor = new CANSparkMax(2, MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(4, MotorType.kBrushless);

    // restore factory defaults
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();

    //set idle mode
    setDriveIdleMode(Constants.idleMode);

    // configure ramping
    leftFrontMotor.setOpenLoopRampRate(Constants.openLoopRampRate);
    leftBackMotor.setOpenLoopRampRate(Constants.openLoopRampRate);
    rightFrontMotor.setOpenLoopRampRate(Constants.openLoopRampRate);
    rightBackMotor.setOpenLoopRampRate(Constants.openLoopRampRate);

    // set current limit
    leftFrontMotor.setSmartCurrentLimit(Constants.currentLimit);
    leftBackMotor.setSmartCurrentLimit(Constants.currentLimit);
    rightFrontMotor.setSmartCurrentLimit(Constants.currentLimit);
    rightBackMotor.setSmartCurrentLimit(Constants.currentLimit);

    // leader/follower config
    leftFrontMotor.follow(leftBackMotor, false); // false means not inverted, and true means inverted
    rightFrontMotor.follow(rightBackMotor, false);

    // differential drive setup
    diffDrive = new DifferentialDrive(leftBackMotor, rightBackMotor);
    diffDrive.setDeadband(0.05); // minmal signal
    rightBackMotor.setInverted(true);
  }

  // motor utils
  private void setDriveIdleMode(IdleMode mode) {
    // IdleMode mode = IdleMode.kBrake; // brakes
    leftFrontMotor.setIdleMode(mode);
    leftBackMotor.setIdleMode(mode);
    rightFrontMotor.setIdleMode(mode);
    rightBackMotor.setIdleMode(mode);
  }

  // sensors
  private void initSensors() {
    // Encoders
    leftBackEncoder = leftBackMotor.getEncoder();
    leftFrontEncoder = leftFrontMotor.getEncoder();
    rightBackEncoder = rightBackMotor.getEncoder();
    rightFrontEncoder = rightFrontMotor.getEncoder();

    // is this used for anything? 
    countsPerMotorRevolution = leftBackEncoder.getCountsPerRevolution();
    // this choice of encoder is arbitrary -- any other encoder would work just as
    // well

    leftBackEncoder.setPositionConversionFactor(Constants.conversionFactor);
    leftFrontEncoder.setPositionConversionFactor(Constants.conversionFactor);
    rightFrontEncoder.setPositionConversionFactor(Constants.conversionFactor);
    rightBackEncoder.setPositionConversionFactor(Constants.conversionFactor);

    resetEncoders();

    // NavX Mini (Gyro)
    navx = new AHRS(SerialPort.Port.kUSB);

    // Pathing Odometry
    odometry = new DifferentialDriveOdometry(navx.getRotation2d());

  }

  /* TELEOP CONTROL */
  // curvature drive
  public void curvatureInput(double speed, double rotation, boolean isCurvatureDrive) {
    // TODO: add constants for speed and rotation
    diffDrive.curvatureDrive(speed, rotation, isCurvatureDrive);
  }

  /* PATHING UTILS */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getDistance(leftBackEncoder), getDistance(rightBackEncoder));
  }

  public double getHeading() {
    return navx.getRotation2d().getDegrees();
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
    odometry.resetPosition(pose, navx.getRotation2d());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftBackMotor.setVoltage(leftVolts);
    rightBackMotor.setVoltage(rightVolts);
    diffDrive.feed();
  }

  // What is this for????
  public CANSparkMax getMotor(int idx) {
    idx = (idx - 1) % 4 + 1;
    switch (idx) {
      case 1:
        return leftFrontMotor;
      case 2:
        return leftBackMotor;
      case 3:
        return rightFrontMotor;
    }
    return rightBackMotor;
  }

  // What are all these for ????
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

  @Override
  public void periodic() {
    odometry.update(navx.getRotation2d(), getDistance(leftBackEncoder), getDistance(rightBackEncoder));
  }

  protected double getDistance(RelativeEncoder enc) {
    return enc.getPosition() * enc.getPositionConversionFactor();
  }
}
