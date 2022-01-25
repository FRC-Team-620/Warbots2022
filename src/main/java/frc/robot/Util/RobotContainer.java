// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import frc.robot.Drive.Drivetrain;
import frc.robot.Drive.RotateMotor;
import frc.robot.Shooter.ShooterCommand;
import frc.robot.Shooter.ShooterSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Drive.DriveWithJoystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
/** Add your docs here. */
public class RobotContainer {
    protected Drivetrain drivetrain = new Drivetrain();
    protected XboxController driver = new XboxController(0);
    protected ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    
    

    protected DriveWithJoystick driveWithJoystick;
    protected RotateMotor rotateMotor;
    protected ShooterCommand shooterCommand;

    //public Command getDriveWithJoystick() {
        //return new DriveWithJoystick(drivetrain, driver);
    //}
    public RobotContainer() {
        
    }

    public void init() {
        //rotateMotor = new RotateMotor(drivetrain, driver);
        driveWithJoystick = new DriveWithJoystick(drivetrain, driver);
        drivetrain.setDefaultCommand(driveWithJoystick);
        rotateMotor = new RotateMotor(drivetrain, driver);
        JoystickButton leftBumper = new JoystickButton(driver, Button.kLeftBumper.value);
        leftBumper.whenPressed(rotateMotor);
        shooterCommand = new ShooterCommand(shooterSubsystem, driver);
        shooterSubsystem.setDefaultCommand(shooterCommand);
        //drivetrain.setDefaultCommand(rotateMotor);
        //rotateMotor.schedule(false);
    } 

    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0.5, 0)),//, new Translation2d(2, -1)//1
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            drivetrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts,
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }


}