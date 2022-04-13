package frc.robot.Auto.Routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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

public class TwoBalls extends SequentialCommandGroup {
  private double twoBallsDistanceMeters = 2;

  public TwoBalls(Drivetrain drivetrain, LazySusanSubsystem lazySusanSubsystem, 
  ShooterSubsystem shooterSubsystem, FiringPins firingPins, Intake intake) {
    addCommands(
      new ParallelCommandGroup(
        new AutoLoad(intake),
        new LimelightSpinUp(shooterSubsystem),
        new SequentialCommandGroup(
          new DriveForwardDistance(drivetrain, twoBallsDistanceMeters),
          new WaitCommand(3),
          new ActivateFiringPins(firingPins, intake),
          new ParallelCommandGroup(
            new WaitCommand(5),
            new InstantCommand(intake::enableInnerIntakeMotor)
          ),
          new WaitCommand(1),
          new ActivateFiringPins(firingPins, intake)
        )
      ), 
      new InstantCommand(intake::disableInnerIntakeMotor)
      // new SetpointSpinUp(shooterSubsystem, LimeLight.getTY()),
      
    );
  }
}
