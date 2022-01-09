// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
  private DifferentialDrive diffDrive;

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

    IdleMode mode = IdleMode.kBrake;//brakes
    leftFrontMotor.setIdleMode(mode);
    leftBackMotor.setIdleMode(mode);
    rightFrontMotor.setIdleMode(mode);
    rightBackMotor.setIdleMode(mode);

    double openLoopRampRate = 0.6;//0.6 sec to full velocity
    leftFrontMotor.setOpenLoopRampRate(openLoopRampRate);
    leftBackMotor.setOpenLoopRampRate(openLoopRampRate);
    rightFrontMotor.setOpenLoopRampRate(openLoopRampRate);
    rightBackMotor.setOpenLoopRampRate(openLoopRampRate);

    int currentLimit = 45;//maxmium amps
    leftFrontMotor.setSmartCurrentLimit(currentLimit);
    leftBackMotor.setSmartCurrentLimit(currentLimit);
    rightFrontMotor.setSmartCurrentLimit(currentLimit);
    rightBackMotor.setSmartCurrentLimit(currentLimit);

    leftFrontMotor.follow(leftBackMotor, false);//false means not inverted, and true means inverted
    rightFrontMotor.follow(rightBackMotor, false);

    diffDrive = new DifferentialDrive(leftBackMotor, rightBackMotor);
    diffDrive.setDeadband(0.05);//minmal signal

  }

  public void curvatureInput(double speed, double rotation, boolean isCurvatureDrive) {
    diffDrive.curvatureDrive(speed, rotation, isCurvatureDrive);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
