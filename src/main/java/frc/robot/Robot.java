// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Util.RobotContainer;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  protected RobotContainer robotContainer;
  protected Command autonomousCommand;
  
  @Override
  public void robotInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer = new RobotContainer();
    robotContainer.init();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    robotContainer.robotFieldWidget.setRobotPose(robotContainer.getDriveTrain().getPose()); //Example on how to update Field2d with robot position.
    //robotContainer.drivetrain.leftFrontMotorDrive(0.3);
  } 

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.init();
  }

  @Override
  public void testPeriodic() {
    //robotContainer.drivetrain.leftFrontMotorDrive(0.3);
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
       autonomousCommand.cancel();
    }
    robotContainer.getLazySusanSubsystem().getLazySusanEncoder().setPosition(0);
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }


  @Override
  public void disabledInit() {
    IdleMode mode = IdleMode.kCoast;
    robotContainer.getDriveTrain().setMotorMode(mode);
  }
  
}
