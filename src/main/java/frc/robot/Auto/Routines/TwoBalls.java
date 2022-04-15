package frc.robot.Auto.Routines;

//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auto.DriveBackwardDistance;
import frc.robot.Auto.DriveForwardDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Loader.AutoLoad;
import frc.robot.Loader.Intake;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LimelightSpinUp;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.TankDriveAiming;

public class TwoBalls extends SequentialCommandGroup {
  private double firstShotDistance = 0.5;
  private double twoBallsDistanceMeters = 1.5;

  public TwoBalls(Drivetrain drivetrain, ShooterSubsystem shooterSubsystem, 
  FiringPins firingPins, Intake intake) {
    addCommands(
      new ParallelCommandGroup(
        new LimelightSpinUp(shooterSubsystem),
        new SequentialCommandGroup(
          // new ParallelCommandGroup(
          //   new AutoLoad(intake),
          //   new DriveForwardDistance(drivetrain, 0.5, intake)
          // ),
          new DriveForwardDistance(drivetrain, firstShotDistance, intake),
          // new DriveForwardDistance(drivetrain, twoBallsDistanceMeters),
          new TankDriveAiming(drivetrain, 300),
          new ActivateFiringPins(firingPins, intake),
          new ParallelCommandGroup(
            new AutoLoad(intake),
            new DriveForwardDistance(drivetrain, twoBallsDistanceMeters, intake)
          ),
          // new ParallelCommandGroup(
          //   new WaitCommand(5),
          //   new InstantCommand(intake::enableInnerIntakeMotor)
          // ),
          new DriveBackwardDistance(drivetrain, twoBallsDistanceMeters),
          new TankDriveAiming(drivetrain, 300),
          new ActivateFiringPins(firingPins, intake)
        )
      ) 
      // new InstantCommand(intake::disableInnerIntakeMotor)
      // new SetpointSpinUp(shooterSubsystem, LimeLight.getTY()),
      
    );
  }
}
