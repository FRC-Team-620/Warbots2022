// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.AutoCommand;
import frc.robot.Climber.LowerHooks;
import frc.robot.Loader.AutoLoad;
import frc.robot.Shooter.AutoAimingAndSpinningUp;
import frc.robot.Shooter.DirectTurretAuto;
import frc.robot.Util.RobotContainer;

public class Robot extends TimedRobot {
  protected RobotContainer robotContainer;
  protected Command autonomousCommand;

  @Override
  public void robotInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer = new RobotContainer();
    robotContainer.init();// TODO: make these happen on RobotContainer instantiation
    robotContainer.getShooterCommand().getTable().getEntry("ledMode").setNumber(1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // TODO: fork off for sim code, do not merge with competition code
    // var robotpos = robotContainer.getDriveTrain().getPose();
    // robotContainer.robotFieldWidget.setRobotPose(robotpos); // Example on how to
    // update Field2d with robot position.
    // // var tmp = robotContainer.getDriveTrain().getPose().plus(new Pose2d(0, 0,
    // // robotContainer.getLazySusanSubsystem().turrentRotation.plus(other));
    // var hubpos = new Pose2d(7.940, 4.08, new Rotation2d());
    // // hubpos.
    // robotContainer.robotFieldWidget.getObject("hub").setPose(hubpos);
    // var tpos = new Pose2d(robotpos.getTranslation(),
    // robotpos.getRotation().plus(robotContainer.getLazySusanSubsystem().simTurrentRotation));
    // robotContainer.robotFieldWidget.getObject("Turret").setPose(tpos);

    // robotContainer.getShooterSubsystem().possim.setPosition(tpos);
    // robotContainer.getShooterSubsystem().possim.update(Constants.kSimUpdateTime);
    // robotContainer.drivetrain.leftFrontMotorDrive(0.3);
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // TODO: move to initializer in robotContainer
    new LowerHooks(robotContainer.getClimberSubsystem()).schedule();
    robotContainer.getLazySusanSubsystem().getLazySusanEncoder().setPosition(0);
    robotContainer.getDriveTrain().setMotorMode(IdleMode.kBrake);
    robotContainer.getClimberMotorsSubsystem().getWinchMotor().getEncoder().setPosition(0);
  }

  @Override
  public void autonomousInit() {

    // TODO: move to autonomousCommand in separate file.
    robotContainer.getLazySusanSubsystem().setLazySusanPosition(0);
    new LowerHooks(robotContainer.getClimberSubsystem()).schedule();
    autonomousCommand = new SequentialCommandGroup(
        new DirectTurretAuto(robotContainer.getLazySusanSubsystem(), // -1.5*
            robotContainer.getShooterSubsystem(), 0),
        new ParallelCommandGroup(
            new AutoAimingAndSpinningUp(robotContainer.getShooterSubsystem(),
                robotContainer.getLazySusanSubsystem(), true),
            new AutoCommand(robotContainer.getLoaderSubsystem(),
                robotContainer.getShooterSubsystem(), robotContainer.getLazySusanSubsystem(), robotContainer),
            new AutoLoad(robotContainer.getLoaderSubsystem(), 1)));

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void disabledInit() {
    IdleMode mode = IdleMode.kBrake;
    robotContainer.getDriveTrain().setMotorMode(mode);
    robotContainer.getShooterSubsystem().setSpeed(0);
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.init();
  }

  @Override
  public void testPeriodic() {
  }
  // TODO: again, sim code separate from competition code.
  // @Override
  // public void simulationPeriodic() {
  // // Here we calculate the battery voltage based on drawn current.
  // // As our robot draws more power from the battery its voltage drops.
  // // The estimated voltage is highly dependent on the battery's internal
  // // resistance.
  // double drawCurrent = robotContainer.getDriveTrain().getDrawnCurrentAmps(); //
  // Current Seems to be too high look into
  // // later
  // drawCurrent += robotContainer.getShooterSubsystem().getDrawnCurrentAmps();
  // SmartDashboard.putNumber("Total Current", drawCurrent);
  // // BatterySim.calculateDefaultBatteryLoadedVoltage(currents)
  // double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(13,
  // 0.02, drawCurrent);
  // SmartDashboard.putNumber("Robot Volts", loadedVoltage);
  // RoboRioSim.setVInVoltage(loadedVoltage);
  // }
}
