// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  protected final CANSparkMax rightBackMotor, leftBackMotor, rightFrontMotor, leftFrontMotor;
  protected final DifferentialDrive diffDrive;
  protected final RelativeEncoder leftBackEncoder, leftFrontEncoder, rightBackEncoder, rightFrontEncoder;
  protected final double countsPerMotorRevolution;
  // The gyro sensor
  protected final Gyro gyro;
  
  // Odometry class for tracking robot pose
  protected final DifferentialDriveOdometry odometry;



  /** Creates a new Drivetrain. */
  public Drivetrain() {
    rightBackMotor = new CANSparkMax(1, MotorType.kBrushless);
    leftBackMotor = new CANSparkMax(3, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
    leftFrontMotor = new CANSparkMax(4, MotorType.kBrushless);

    rightBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    leftFrontMotor.restoreFactoryDefaults();

    IdleMode mode = IdleMode.kBrake; //brakes
    rightBackMotor.setIdleMode(mode);
    rightFrontMotor.setIdleMode(mode);
    leftBackMotor.setIdleMode(mode);
    leftFrontMotor.setIdleMode(mode);

    double openLoopRampRate = 0.6; //0.6 sec to full velocity
    rightBackMotor.setOpenLoopRampRate(openLoopRampRate);
    rightFrontMotor.setOpenLoopRampRate(openLoopRampRate);
    leftBackMotor.setOpenLoopRampRate(openLoopRampRate);
    leftFrontMotor.setOpenLoopRampRate(openLoopRampRate);

    int currentLimit = 45; //maxmium amps
    rightBackMotor.setSmartCurrentLimit(currentLimit);
    rightFrontMotor.setSmartCurrentLimit(currentLimit);
    leftBackMotor.setSmartCurrentLimit(currentLimit);
    leftFrontMotor.setSmartCurrentLimit(currentLimit);

    rightBackMotor.follow(rightFrontMotor, false); //false means not inverted, and true means inverted
    leftBackMotor.follow(leftFrontMotor, false);

    // Encoder creation
    leftBackEncoder = rightFrontMotor.getEncoder();
    leftFrontEncoder = rightBackMotor.getEncoder();
    rightBackEncoder = leftFrontMotor.getEncoder();
    rightFrontEncoder = leftBackMotor.getEncoder();

    countsPerMotorRevolution = leftBackEncoder.getCountsPerRevolution(); 
    //this choice of encoder is arbitrary -- any other encoder would work just as well

    var conversionFactor = Constants.gearRatio * Constants.wheelDiameterInInches * Constants.inchesToMetersFactor * Math.PI;
    leftBackEncoder.setPositionConversionFactor(conversionFactor);
    leftFrontEncoder.setPositionConversionFactor(conversionFactor);
    rightFrontEncoder.setPositionConversionFactor(conversionFactor);
    rightBackEncoder.setPositionConversionFactor(conversionFactor);

    gyro = new AHRS(SerialPort.Port.kUSB);

    diffDrive = new DifferentialDrive(rightFrontMotor, leftFrontMotor);
    diffDrive.setDeadband(0.05); //minmal signal
    leftFrontMotor.setInverted(true);
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
    rightFrontMotor.setVoltage(leftVolts);
    leftFrontMotor.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public CANSparkMax getMotor(int idx) {
    idx = (idx-1)%4+1;
    switch(idx) {
      case 1:
        return rightBackMotor;
      case 2:
        return rightFrontMotor;
      case 3:
        return leftBackMotor;
    }
    return leftFrontMotor;
  }

  public void curvatureInput(double speed, double rotation, boolean isCurvatureDrive) {
    diffDrive.curvatureDrive(speed, rotation, isCurvatureDrive);
  }

  public void leftBackMotorDrive(double x) {
    rightFrontMotor.set(x);
  }

  public void rightBackMotorDrive(double x) {
    leftFrontMotor.set(x);
  }

  public void leftFrontMotorDrive(double x) {
    rightBackMotor.set(x);
  }

  public void rightFrontMotorDrive(double x) {
    leftBackMotor.set(x);
  }
  
  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getDistance(leftBackEncoder), getDistance(rightBackEncoder));
  }

  protected double getDistance(RelativeEncoder enc) {
    return enc.getPosition() * enc.getPositionConversionFactor();
  }
}
