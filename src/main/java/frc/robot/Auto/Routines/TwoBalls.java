package frc.robot.Auto.Routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auto.DriveBackwardDistance;
import frc.robot.Auto.DriveForwardDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Loader.AutoLoad;
import frc.robot.Loader.Intake;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.LimelightSpinUp;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.TurretAimingPID;
import frc.robot.Shooter.ZeroTurnTable;
import frc.robot.Util.LimeLight;
import frc.robot.Util.RobotContainer;
import frc.robot.Util.LimeLight.LedMode;

public class TwoBalls extends SequentialCommandGroup {
  // All distances are in meters
  private double firstShotDistance = 0.5;
  private double twoBallsDistance = 0.8;
  private double taxiDistance = 1.2;

  public TwoBalls(RobotContainer robotContainer, Drivetrain drivetrain, LazySusanSubsystem lazySusanSubsystem, 
  ShooterSubsystem shooterSubsystem, FiringPins firingPins, Intake intake) {
    if (!lazySusanSubsystem.getIsCal()) {
      addCommands(
      parallel(
        new LimelightSpinUp(shooterSubsystem),
        sequence(
          // parallel(
          //   new AutoLoad(intake),
          //   new DriveForwardDistance(drivetrain, 0.5, intake)
          // ),
          parallel(
            new ZeroTurnTable(lazySusanSubsystem),
            new DriveForwardDistance(drivetrain, firstShotDistance, intake)
          ),
          new WaitCommand(3),
          // new DriveForwardDistance(drivetrain, twoBallsDistanceMeters),
          new TurretAimingPID(lazySusanSubsystem, robotContainer.robotFieldWidget, drivetrain::getPose, 100, false),
          new ActivateFiringPins(firingPins, intake),
          parallel(
            new AutoLoad(intake),
            new DriveForwardDistance(drivetrain, twoBallsDistance, intake)
          ),
          // parallel(
          //   new WaitCommand(5),
          //   new InstantCommand(intake::enableInnerIntakeMotor)
          // ),
          new DriveBackwardDistance(drivetrain, twoBallsDistance),
          new TurretAimingPID(lazySusanSubsystem, robotContainer.robotFieldWidget, drivetrain::getPose, 100, false),
          new ActivateFiringPins(firingPins, intake),

          new WaitCommand(1),
          new DriveForwardDistance(drivetrain, taxiDistance, intake)
        )
      ),
      // new InstantCommand(intake::disableInnerIntakeMotor)
      // new SetpointSpinUp(shooterSubsystem, LimeLight.getTY()),
      new InstantCommand(() -> LimeLight.setLedMode(LedMode.OFF))
    );
    } else {
      addCommands(
      parallel(
        new LimelightSpinUp(shooterSubsystem),
        sequence(
          // parallel(
          //   new AutoLoad(intake),
          //   new DriveForwardDistance(drivetrain, 0.5, intake)
          // ),
          new DriveForwardDistance(drivetrain, firstShotDistance, intake),
          // new DriveForwardDistance(drivetrain, twoBallsDistanceMeters),
          new WaitCommand(3),
          new TurretAimingPID(lazySusanSubsystem, robotContainer.robotFieldWidget, drivetrain::getPose, 100, true),
          new ActivateFiringPins(firingPins, intake),
          parallel(
            new AutoLoad(intake),
            new DriveForwardDistance(drivetrain, twoBallsDistance, intake)
          ),
          // parallel(
          //   new WaitCommand(5),
          //   new InstantCommand(intake::enableInnerIntakeMotor)
          // ),
          new DriveBackwardDistance(drivetrain, twoBallsDistance),
          new TurretAimingPID(lazySusanSubsystem, robotContainer.robotFieldWidget, drivetrain::getPose, 100, true),
          new ActivateFiringPins(firingPins, intake),

          new WaitCommand(1),
          new DriveForwardDistance(drivetrain, taxiDistance, intake)
        )
      ),
      // new InstantCommand(intake::disableInnerIntakeMotor)
      // new SetpointSpinUp(shooterSubsystem, LimeLight.getTY()),
      new InstantCommand(() -> LimeLight.setLedMode(LedMode.OFF))
     );
    }
    
  }
}
