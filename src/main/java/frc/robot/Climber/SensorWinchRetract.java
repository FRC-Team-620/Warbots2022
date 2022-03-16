// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class SensorWinchRetract extends CommandBase {
  protected ClimberMotorsSubsystem climberMotorsSubsystem;
  protected ClimberSubsystem climberSubsystem;
  protected double deltaCounts, targetCounts;

  public SensorWinchRetract(ClimberMotorsSubsystem climberMotorsSubsystem, ClimberSubsystem climberSubsystem) {
      addRequirements(climberMotorsSubsystem);
      this.climberMotorsSubsystem = climberMotorsSubsystem;
      this.climberSubsystem = climberSubsystem;
  }

  @Override
  public void initialize() {
      System.out.println("Current: " + this.climberMotorsSubsystem.getWinchPosition());
      this.climberMotorsSubsystem.setWinchSpeed(-1);
      // System.out.println("Winch begins wind up");
  }

  @Override
  public void execute() {
      System.out.println("Winch is winding");
      // this.climberMotorsSubsystem.setWinchSpeed(
      //     Math.abs(this.targetCounts-this.climberMotorsSubsystem.getWinchPosition())>10?-1:-.2);
  }

  @Override
  public void end(boolean interrupted) {
      this.climberMotorsSubsystem.setWinchSpeed(0);
  }

  @Override
  public boolean isFinished() {//climberSubsystem.getWinchMotor().getEncoder().getPosition() <= -counts
      System.out.println("Climber Sensor Hitting: " + climberSubsystem.getClimberSensor());
      return climberSubsystem.getClimberSensor();
  }
}
