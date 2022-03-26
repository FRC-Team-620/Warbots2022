// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive.Drivetrain;
import frc.robot.Drive.DriveForwardsEncoder;

public class DriveForwardDistance extends CommandBase {
  Drivetrain drivetrain;
  private double distance;
  private double autoSpeed = 0.5;

  // protected final PIDController leftPID;
  // protected final PIDController rightPID;
  /** Creates a new DriveForwardDIstance. */
  public DriveForwardDistance(Drivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.distance = distance;
    // leftShooterPID = new PIDController(kP, kI, 0);
    // rightShooterPID = new PIDController(kP, kI, 0);
    addRequirements(drivetrain);
  }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDriveSet(autoSpeed, 0);
  }

  public boolean withinBounds() {
    if (averageDistance() > distance) {
      return true;
    }
    return false;
  }

  public double averageDistance() {
    return (drivetrain.getLeftPosition() + drivetrain.getRightPosition()) / 2;

  }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return withinBounds();
  }

  @Override
  public void end(boolean interrupt) {
    drivetrain.arcadeDriveSet(0, 0);
  }
}
