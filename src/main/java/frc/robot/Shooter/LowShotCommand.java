// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Controls.ControlBoard;



public class LowShotCommand extends CommandBase {
  protected XboxController operatorXbox;
    protected XboxController driverXbox;
    protected ShooterSubsystem shooterSubsystem;
    protected LazySusanSubsystem lazySusanSubsystem;
    protected ControlBoard controlBoard;
  protected double acceleration; 
  /** Creates a new LowShotCommand. */
  public LowShotCommand(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setTargetRPM(Constants.lowPoweredShotRPM);
    // acceleration = Constants.diffConstShooter * (shooterSubsystem.getTargetRPM() - shooterSubsystem.getRPM());
    //     SmartDashboard.putNumber("Acceleration: ", acceleration);
    //     // if(Math.abs(rpm-currRPM) > (bangBangTolerance * rpm))
    //     shooterSubsystem.setShooterSpeedAndUpdate(shooterSubsystem.getCurrentSpeed() + acceleration);
    //     // System.out.println(rpm);
    //     if(shooterSubsystem.getTargetRPM() == 0.0) {
    //         shooterSubsystem.setShooterSpeedAndUpdate(0);
    //     }
    //     SmartDashboard.putNumber("Shooter RPM: ",
    //             ShooterMath.roundUpToNearestMultiple(shooterSubsystem.getRPM(), 5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
