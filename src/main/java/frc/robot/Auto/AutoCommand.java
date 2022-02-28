package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Loader.AutoLoad;
import frc.robot.Loader.AutoShoot;
import frc.robot.Loader.LoaderSubsystem;
import frc.robot.Shooter.AutoAimingAndSpinningUp;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Util.RobotContainer;

public class AutoCommand extends SequentialCommandGroup {
    LoaderSubsystem loaderSubsystem;
    ShooterSubsystem shooterSubsystem;
    LazySusanSubsystem lazySusanSubsystem;
    RobotContainer robotContainer;
    public AutoCommand(LoaderSubsystem lS, ShooterSubsystem sS, LazySusanSubsystem lSS, RobotContainer rC) {
        this.loaderSubsystem = lS;
        this.shooterSubsystem = sS;
        this.lazySusanSubsystem = lSS;
        this.robotContainer = rC;
        addRequirements(loaderSubsystem, shooterSubsystem, lazySusanSubsystem);
        
        AutoLoad autoLoad = new AutoLoad(loaderSubsystem, 1);
        AutoAimingAndSpinningUp autoAimingAndSpinningUp = new AutoAimingAndSpinningUp(shooterSubsystem, lazySusanSubsystem);
        shooterSubsystem.setDefaultCommand(autoAimingAndSpinningUp);
        addCommands(
            autoLoad,

            robotContainer.getAutonomousCommand(robotContainer.getTrajectorySelector().getPart1()),

            new AutoShoot(loaderSubsystem),

            robotContainer.getAutonomousCommand(robotContainer.getTrajectorySelector().getPart2()),

            new AutoShoot(loaderSubsystem),

            robotContainer.getAutonomousCommand(robotContainer.getTrajectorySelector().getPart3()),

            new AutoShoot(loaderSubsystem),

            robotContainer.getAutonomousCommand(robotContainer.getTrajectorySelector().getPart4()),

            new AutoShoot(loaderSubsystem),

            new AutoLoad(loaderSubsystem, 0)
            
        );
        
    }
}
