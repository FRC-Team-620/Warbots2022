// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;
import frc.robot.Util.RobotContainer;

public class Robot extends TimedRobot {
  protected RobotContainer robotContainer;
  protected Command autonomousCommand;
  protected int LEDDisplacement = 0;

  @Override
  public void robotInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer = new RobotContainer();
    robotContainer.init();
  }

  @Override
  public void robotPeriodic() {
  	CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    LimeLight.setLedMode(LedMode.OFF);
    robotContainer.getIntake().disableInnerIntakeMotor();
    robotContainer.getShooterSubsystem().setOffsetSpeed(0);
    robotContainer.setTeleopDrive();

    robotContainer.getDriveTrain().setBrake(true);
  }

  @Override
  public void autonomousInit() {
    LimeLight.setLedMode(LedMode.OFF);
    robotContainer.getShooterSubsystem().setOffsetSpeed(0);
  }

  @Override
  public void autonomousExit() {
    LimeLight.setLedMode(LedMode.OFF);
  }

  @Override
  public void disabledInit() {
    robotContainer.getDriveTrain().setBrake(true);
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.init();
  }
}
