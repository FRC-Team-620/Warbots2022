package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auto.Routines.Taxi;
import frc.robot.Drive.DriveForwardsEncoder;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.ZeroTurnTable;
import frc.robot.Util.RobotContainer;
import frc.robot.Util.WaitFrames;

public class AutoCommand extends SequentialCommandGroup {
    FiringPins firingPins;
    ShooterSubsystem shooterSubsystem;
    LazySusanSubsystem lazySusanSubsystem;
    RobotContainer robotContainer;
    
    public AutoCommand(FiringPins fP, ShooterSubsystem sS, LazySusanSubsystem lSS, RobotContainer rC) {
        this.firingPins = fP;
        this.shooterSubsystem = sS;
        this.lazySusanSubsystem = lSS;
        this.robotContainer = rC;
        // addRequirements(loaderSubsystem, shooterSubsystem, lazySusanSubsystem);
            addCommands(
            new ConditionalCommand(new WaitCommand(0), new ZeroTurnTable(lazySusanSubsystem), lazySusanSubsystem::getIsCal),
            new Taxi(robotContainer.getDriveTrain())
            );
        
    }
}
