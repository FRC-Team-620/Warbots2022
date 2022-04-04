// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auto.DriveForwardDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Loader.AutoLoad;
import frc.robot.Loader.Intake;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.LimelightSpinUp;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Util.WaitFrames;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBalls extends SequentialCommandGroup {
  /** Creates a new OneBall. */
    Drivetrain drivetrain;
    LazySusanSubsystem lazySusanSubsystem;
    ShooterSubsystem shooterSubsystem;
    FiringPins firingPins;
    Intake intake;

    private double twoBallsDistanceMeters = 2;
  public TwoBalls(Drivetrain drivetrain, LazySusanSubsystem lazySusanSubsystem, ShooterSubsystem shooterSubsystem, FiringPins firingPins, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.drivetrain = drivetrain;
    this.lazySusanSubsystem = lazySusanSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.firingPins = firingPins;
    this.intake = intake;
    addCommands(
      new ParallelCommandGroup(
        new AutoLoad(intake),
        new LimelightSpinUp(shooterSubsystem),
        new SequentialCommandGroup(
          new DriveForwardDistance(drivetrain, twoBallsDistanceMeters),
          new WaitCommand(1),
          new ActivateFiringPins(firingPins),
          new WaitCommand(5),
          new ActivateFiringPins(firingPins)
        )
      )
      // new SetpointSpinUp(shooterSubsystem, LimeLight.getTY()),
      
    );
  }
}
