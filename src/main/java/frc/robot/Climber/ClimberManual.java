// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class ClimberManual extends CommandBase {
  /** Creates a new ClimberManual. */
  protected ClimberMotorsSubsystem climberMotorsSubsystem;
  protected XboxController operatorController;
  protected double winchSpeed;
  public ClimberManual(ClimberMotorsSubsystem climberMotorsSubsystem, XboxController operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberMotorsSubsystem = climberMotorsSubsystem; 
    this.operatorController = operatorController;
    addRequirements(climberMotorsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //int dpadDir = operatorController.getPOV();
    // if (climberMotorsSubsystem.getWinchMotor().getEncoder().getPosition() >= 30) {

    // }
    // if(climberMotorsSubsystem.getWinchPosition() < Constants.winchLimit) {
    //   if (operatorController.getLeftStickButton()) {
    //     climberMotorsSubsystem.setWinchSpeed(0.8);
    //   } else if (operatorController.getBackButton()) {
    //     climberMotorsSubsystem.setWinchSpeed(0.1);
    //   }
    // }
    // if(climberMotorsSubsystem.getWinchPosition() > 0) {
    //   if(operatorController.getRightStickButton()) {
    //     climberMotorsSubsystem.setWinchSpeed(-0.8);
    //   }
    // }
    if (operatorController.getLeftStickButton()) {
      winchSpeed = 0.8;
    } else if (operatorController.getRightStickButton()) {
      winchSpeed = -0.8;
    } else if (operatorController.getBackButton()) {
      winchSpeed = 0.1;
    } else {
      winchSpeed = 0;
    }
    if (climberMotorsSubsystem.getClimberSensor()) {
      winchSpeed = winchSpeed * 0.5;
    }
    // // Winch soft limits -- BE SURE TO START WITH THE CLIMBER IN THE MINIMUM POSITION
    // if((climberMotorsSubsystem.getWinchPosition() >= Constants.winchMaxLimit && winchSpeed > 0) || 
    //  (climberMotorsSubsystem.getWinchPosition() <= Constants.winchMinLimit && winchSpeed < 0)) {
    //   winchSpeed = 0;
    // }
    climberMotorsSubsystem.setWinchSpeed(winchSpeed);
    //System.out.println("Winch Position: " + climberMotorsSubsystem.getWinchMotor().getEncoder().getPosition());
    // if (dpadDir == 0) {
    //   climberSubsystem.setWinchSpeed(1);//DPAD up
    // } else if (dpadDir == 180) {
    //   climberSubsystem.setWinchSpeed(-1);//DPAD down
    // } else {
    //   climberSubsystem.setWinchSpeed(0);//DPAD nothing
    // }
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
