package frc.robot.Auto.Routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auto.DriveForwardDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Loader.Intake;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.LimelightSpinUp;
import frc.robot.Shooter.ShooterSubsystem;

public class OneBall extends SequentialCommandGroup {
    private double oneBallDistanceMeters = 2;
  public OneBall(Drivetrain drivetrain, LazySusanSubsystem lazySusanSubsystem, 
  ShooterSubsystem shooterSubsystem, FiringPins firingPins, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveForwardDistance(drivetrain, oneBallDistanceMeters),
      // new SetpointSpinUp(shooterSubsystem, LimeLight.getTY()),
      new WaitCommand(1),
      new LimelightSpinUp(shooterSubsystem),
      //new InstantCommand(() -> shooterSubsystem.setTargetRPM(LimeLight.getTY()), shooterSubsystem),
      new ActivateFiringPins(firingPins, intake)
      );
  }
}
