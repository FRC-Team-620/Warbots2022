// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;
import frc.robot.Util.RobotContainer;

public class Robot extends TimedRobot {
  protected RobotContainer robotContainer;
  protected Command autonomousCommand;
  protected SendableChooser<CommandBase> autoSelector = new SendableChooser<CommandBase>();
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
    robotContainer.getClimberMotorsSubsystem().getWinchMotor().getEncoder().setPosition(0);
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
  public void teleopPeriodic() {
    SmartDashboard.putBoolean("Rear Limit Switch", robotContainer.getClimberMotorsSubsystem().hitRearLimitSwitch());
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

  /**
   * Simulation Code
   */
  @Override
  public void simulationPeriodic() {
    // // Here we calculate the battery voltage based on drawn current.
    // // As our robot draws more power from the battery its voltage drops.
    // // The estimated voltage is highly dependent on the battery's internal
    // // resistance.
    // double drawCurrent = robotContainer.getDriveTrain().getDrawnCurrentAmps(); //
    // // Current Seems to be too high look into
    // // later
    // drawCurrent += robotContainer.getShooterSubsystem().getDrawnCurrentAmps();
    // SmartDashboard.putNumber("Robot/Total Current", drawCurrent);
    // // BatterySim.calculateDefaultBatteryLoadedVoltage(currents)
    // double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(13,
    //     0.02, drawCurrent);
    // SmartDashboard.putNumber("Robot/Robot Volts", loadedVoltage);
    // RoboRioSim.setVInVoltage(loadedVoltage);

    // var robotpos = robotContainer.getDriveTrain().getPose();
    // robotContainer.robotFieldWidget.setRobotPose(robotpos);
    // var hubpos = new Pose2d(7.940, 4.08, new Rotation2d()); // Position of the hub
    // robotContainer.robotFieldWidget.getObject("hub").setPose(hubpos);
    // var tpos = new Pose2d(robotpos.getTranslation(),
    //     robotpos.getRotation().plus(robotContainer.getLazySusanSubsystem().getRotation()));
    //                                                                                              // turrent position
    // robotContainer.robotFieldWidget.getObject("Turret").setPose(tpos);

    // robotContainer.getShooterSubsystem().possim.setPosition(tpos);
    // robotContainer.getShooterSubsystem().possim.update(Constants.kSimUpdateTime);
  }
}
