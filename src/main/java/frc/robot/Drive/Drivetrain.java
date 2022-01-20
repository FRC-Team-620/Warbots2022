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
  private CANSparkMax leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
  private DifferentialDrive diffDrive;

  // The left-side drive encoder
  private final RelativeEncoder leftEncoder = leftBackMotor.getEncoder();

  // The right-side drive encoder
  private final RelativeEncoder rightEncoder = rightBackMotor.getEncoder();


  // The gyro sensor
  private final Gyro m_gyro = new AHRS(SerialPort.Port.kUSB);
  //AHRS gyro = ;
  
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;



  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFrontMotor = new CANSparkMax(1, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(3, MotorType.kBrushless);
    leftBackMotor = new CANSparkMax(2, MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(4, MotorType.kBrushless);

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

    diffDrive = new DifferentialDrive(leftBackMotor, rightBackMotor);
    diffDrive.setDeadband(0.05); //minmal signal
    rightBackMotor.setInverted(true);
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    resetEncoders();
    //leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    //rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);

    //leftEncoder.setPositionConversionFactor(factor)

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity() / 60 * 0.12 * 4 * 0.0254 * Math.PI, rightEncoder.getVelocity() / 60 * 0.12 * 4 * 0.0254 * Math.PI);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
   /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.+
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
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

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
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
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_gyro.getRotation2d(), Drivetrain.findRelativeEncoderDistance(leftEncoder, 4), 
      Drivetrain.findRelativeEncoderDistance(rightEncoder, 4));
  }//6 on main bot, 4 on test bot

  // A helper method which finds the total distance travelled (ideally) by a wheel using a RelativeEncoder
  // wheelDiameter is the diameter of the wheels and is measured in inches
  // this method returns a value in meters
  private static double findRelativeEncoderDistance(RelativeEncoder enc, double wheelDiameter) {
    // 0.12 refers to wheel rotations per motor cycle
    // 0.0254 inches to meters convertion factor 
    return enc.getPosition() * wheelDiameter * Math.PI * 0.12 * 0.0254;
  }
}
