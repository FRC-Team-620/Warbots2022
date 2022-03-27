// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Auto.DriveForwardDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.SetpointSpinUp;
import frc.robot.Shooter.ShooterMath;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Util.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBall extends SequentialCommandGroup {
  /** Creates a new OneBall. */
    Drivetrain drivetrain;
    LazySusanSubsystem lazySusanSubsystem;
    ShooterSubsystem shooterSubsystem;
    FiringPins firingPins;

    private double oneBallDistanceMeters = 2;
    private double y = LimeLight.getTY();
  public OneBall(Drivetrain drivetrain, LazySusanSubsystem lazySusanSubsystem, ShooterSubsystem shooterSubsystem, FiringPins firingPins) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.drivetrain = drivetrain;
    this.lazySusanSubsystem = lazySusanSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.firingPins = firingPins;
    addCommands(
      new DriveForwardDistance(drivetrain, oneBallDistanceMeters),
      new SetpointSpinUp(shooterSubsystem, ShooterMath.getDistanceInMeters(Constants.azimuthAngle1, y, Constants.limelightHeight, Constants.hubHeight)),
      new ActivateFiringPins(firingPins)
      );
  }
}
