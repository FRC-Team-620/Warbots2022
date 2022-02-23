// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Util.RobotContainer;


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
    robotContainer.getDriveTrain().setMotorMode(IdleMode.kBrake);
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void autonomousInit() {
    robotContainer.getLoaderCommand().setAutoOn(true);
    autonomousCommand = robotContainer.getAutonomousCommand(robotContainer.getTrajectorySelector().getPart1());
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    robotContainer.getShooterCommand().setAutoOn(true);
    try {
      wait(2000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    robotContainer.getLoaderCommand().setAutoFire(true);
    try {
      wait(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    robotContainer.getShooterCommand().setAutoOn(false);

    autonomousCommand = robotContainer.getAutonomousCommand(robotContainer.getTrajectorySelector().getPart2());
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    robotContainer.getShooterCommand().setAutoOn(true);
    try {
      wait(2000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    robotContainer.getLoaderCommand().setAutoFire(true);
    try {
      wait(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    robotContainer.getShooterCommand().setAutoOn(false);

    autonomousCommand = robotContainer.getAutonomousCommand(robotContainer.getTrajectorySelector().getPart3());
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    robotContainer.getShooterCommand().setAutoOn(true);
    try {
      wait(2000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    robotContainer.getLoaderCommand().setAutoFire(true);
    try {
      wait(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    robotContainer.getShooterCommand().setAutoOn(false);

    autonomousCommand = robotContainer.getAutonomousCommand(robotContainer.getTrajectorySelector().getPart4());
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    robotContainer.getShooterCommand().setAutoOn(true);
    try {
      wait(2000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    robotContainer.getLoaderCommand().setAutoFire(true);
    try {
      wait(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    robotContainer.getShooterCommand().setAutoOn(false);
    

  }

  @Override
  public void simulationPeriodic() {
    // Here we calculate the battery voltage based on drawn current.
    // As our robot draws more power from the battery its voltage drops.
    // The estimated voltage is highly dependent on the battery's internal
    // resistance.
    double drawCurrent = robotContainer.getDriveTrain().getDrawnCurrentAmps(); //Current Seems to be too high look into later
    drawCurrent += robotContainer.getShooterSubsystem().getDrawnCurrentAmps();
    SmartDashboard.putNumber("Total Current", drawCurrent);
    // BatterySim.calculateDefaultBatteryLoadedVoltage(currents)
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(13,0.02, drawCurrent);
    SmartDashboard.putNumber("Robot Volts", loadedVoltage);
    RoboRioSim.setVInVoltage(loadedVoltage);
  }


  @Override
  public void disabledInit() {
    IdleMode mode = IdleMode.kCoast;
    robotContainer.getDriveTrain().setMotorMode(mode);
    robotContainer.getShooterSubsystem().setShooterSpeed(0);
  }
}
