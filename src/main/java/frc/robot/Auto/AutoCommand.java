package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Drive.DriveForwardsEncoder;
import frc.robot.Loader.AutoShoot;
import frc.robot.Loader.LoaderSubsystem;
import frc.robot.Shooter.DirectTurretAuto;
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
        // addRequirements(loaderSubsystem, shooterSubsystem, lazySusanSubsystem);
            addCommands(
            //robotContainer.getAutonomousCommand(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Part1.wpilib.json"))),
                // new DriveForwards(robotContainer.getDriveTrain()),
                new DriveForwardsEncoder(robotContainer.getDriveTrain(), 2.7),

                //new TurnDegrees(robotContainer.getDriveTrain(), 180),

                new WaitCommand(6),

                new AutoShoot(loaderSubsystem),

                new WaitCommand(3),

                // new DriveForwardsEncoder(robotContainer.getDriveTrain(), 0.3),

                // new WaitCommand(1),

                new AutoShoot(loaderSubsystem),

                new WaitCommand(3)
                

                // robotContainer.getAutonomousCommand(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Part2.wpilib.json"))),

                // new AutoShoot(loaderSubsystem),

                // robotContainer.getAutonomousCommand(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Part3.wpilib.json"))),

                // new AutoShoot(loaderSubsystem),

                // robotContainer.getAutonomousCommand(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Part4.wpilib.json"))),

                // new AutoShoot(loaderSubsystem)
                
                
            );
        
    }
}
