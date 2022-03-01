// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.RobotContainer;

public class ClimberManual extends CommandBase {
  /** Creates a new ClimberManual. */
  protected ClimberSubsystem climberSubsystem;
  protected XboxController operatorController;
  public ClimberManual(ClimberSubsystem climberSubsystem, XboxController operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberSubsystem = climberSubsystem; 
    this.operatorController = operatorController;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int dpadDir = operatorController.getPOV();
    if (dpadDir == 0) {
      climberSubsystem.setWinchSpeed(1);//DPAD up
    } else if (dpadDir == 180) {
      climberSubsystem.setWinchSpeed(-1);//DPAD down
    } else {
      climberSubsystem.setWinchSpeed(0);//DPAD nothing
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
