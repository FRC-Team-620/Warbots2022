// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.AutoCommand;
import frc.robot.Climber.ClimberMotorsSubsystem;
import frc.robot.Climber.ToggleHooks;
import frc.robot.Climber.SensorHooksUp;
import frc.robot.Loader.AutoLoad;
import frc.robot.Shooter.AutoAimingAndSpinningUp;
import frc.robot.Util.RobotContainer;

public class Robot extends TimedRobot {
  protected RobotContainer robotContainer;
  protected Command autonomousCommand;

  @Override
  public void robotInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer = new RobotContainer();
    robotContainer.init();// TODO: make these happen on RobotContainer instantiation
    // robotContainer.getLoaderSubsystem().getExtensionSolenoid().set(true);
    // robotContainer.getLoaderSubsystem().getExtensionSolenoid().set(false);
    // robotContainer.getShooterCommand().getTable().getEntry("ledMode").setNumber(1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //System.out.println(robotContainer.getClimberMotorsSubsystem().getClimberSensor());
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.setTeleopDrive();
    robotContainer.getLoaderSubsystem().setIsClimbing(false);
    // TODO: move to initializer in robotContainer
    new ToggleHooks(robotContainer.getClimberSubsystem()).schedule();
    robotContainer.getLazySusanSubsystem().getLazySusanEncoder().setPosition(0);
    robotContainer.getDriveTrain().setMotorMode(IdleMode.kBrake);
    robotContainer.getClimberMotorsSubsystem().getWinchMotor().getEncoder().setPosition(0);
    // robotContainer.getLoaderSubsystem().getExtensionSolenoid().set(true);
    // robotContainer.getLoaderSubsystem().getExtensionSolenoid().set(false);
  }

  @Override
  public void autonomousInit() {

    // TODO: move to autonomousCommand in separate file.
    robotContainer.getLazySusanSubsystem().setLazySusanPosition(0);
    new ToggleHooks(robotContainer.getClimberSubsystem()).schedule();
    //new DirectTurretAuto(robotContainer.getLazySusanSubsystem(), // -1.5*
    //robotContainer.getShooterSubsystem(), 0),
    autonomousCommand = new SequentialCommandGroup( 
             new ParallelCommandGroup(new AutoCommand(robotContainer.getLoaderSubsystem(), robotContainer.getShooterSubsystem(), robotContainer.getLazySusanSubsystem(), robotContainer), 
                new AutoAimingAndSpinningUp(robotContainer.getShooterSubsystem(),  robotContainer.getLazySusanSubsystem(), true, robotContainer.getOperatorController()), 
                new AutoLoad(robotContainer.getLoaderSubsystem()))
              );

    // autonomousCommand = new SequentialCommandGroup(
    //     new TurnDegrees(robotContainer.getDriveTrain(), 180),
    //     new TurnDegrees(robotContainer.getDriveTrain(), -180),
    //     new TurnDegrees(robotContainer.getDriveTrain(), 360),
    //     new TurnDegrees(robotContainer.getDriveTrain(), -360));
    // new LowerHooks(robotContainer.getClimberSubsystem()).schedule();
    // autonomousCommand = new SequentialCommandGroup(
    // new DirectTurretAuto(robotContainer.getLazySusanSubsystem(), // -1.5*
    // robotContainer.getShooterSubsystem(), 0),
    // new ParallelCommandGroup(
    // new AutoAimingAndSpinningUp(robotContainer.getShooterSubsystem(),
    // robotContainer.getLazySusanSubsystem(), true,
    // robotContainer.getOperatorController()),
    // new AutoCommand(robotContainer.getLoaderSubsystem(),
    // robotContainer.getShooterSubsystem(), robotContainer.getLazySusanSubsystem(),
    // robotContainer),
    // new AutoLoad(robotContainer.getLoaderSubsystem(), 1)));
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
    // new LowerHooks(robotContainer.getClimberSubsystem()).schedule();
    // new SensorHooksUp(robotContainer.getClimberMotorsSubsystem(), robotContainer.getClimberSubsystem()).schedule();
  }

  @Override
  public void testPeriodic() {
  }


  /**
   * Simulation Code
   */
  @Override
  public void simulationPeriodic() {
    // Here we calculate the battery voltage based on drawn current.
    // As our robot draws more power from the battery its voltage drops.
    // The estimated voltage is highly dependent on the battery's internal
    // resistance.
    double drawCurrent = robotContainer.getDriveTrain().getDrawnCurrentAmps(); //
    // Current Seems to be too high look into
    // later
    drawCurrent += robotContainer.getShooterSubsystem().getDrawnCurrentAmps();
    SmartDashboard.putNumber("Total Current", drawCurrent);
    // BatterySim.calculateDefaultBatteryLoadedVoltage(currents)
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(13,
        0.02, drawCurrent);
    SmartDashboard.putNumber("Robot Volts", loadedVoltage);
    RoboRioSim.setVInVoltage(loadedVoltage);

    var robotpos = robotContainer.getDriveTrain().getPose();
    robotContainer.robotFieldWidget.setRobotPose(robotpos);
    var hubpos = new Pose2d(7.940, 4.08, new Rotation2d()); // Position of the hub
    robotContainer.robotFieldWidget.getObject("hub").setPose(hubpos);
    var tpos = new Pose2d(robotpos.getTranslation(),
        robotpos.getRotation().plus(robotContainer.getLazySusanSubsystem().simTurrentRotation)); // Calculate Simulated
                                                                                                 // turrent position
    robotContainer.robotFieldWidget.getObject("Turret").setPose(tpos);

    robotContainer.getShooterSubsystem().possim.setPosition(tpos);
    robotContainer.getShooterSubsystem().possim.update(Constants.kSimUpdateTime);
  }
}
