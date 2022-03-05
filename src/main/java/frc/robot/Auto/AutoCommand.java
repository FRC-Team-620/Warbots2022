package frc.robot.Auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Drive.DriveBackwards;
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
        // addRequirements(loaderSubsystem, shooterSubsystem, lazySusanSubsystem);
            addCommands(
                //robotContainer.getAutonomousCommand(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Part1.wpilib.json"))),
                new DriveBackwards(robotContainer.getDriveTrain()),

                new WaitCommand(0.5),

                new AutoShoot(loaderSubsystem)

                // robotContainer.getAutonomousCommand(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Part2.wpilib.json"))),

                // new AutoShoot(loaderSubsystem),

                // robotContainer.getAutonomousCommand(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Part3.wpilib.json"))),

                // new AutoShoot(loaderSubsystem),

                // robotContainer.getAutonomousCommand(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Part4.wpilib.json"))),

                // new AutoShoot(loaderSubsystem)
                
            );
        
    }
}
