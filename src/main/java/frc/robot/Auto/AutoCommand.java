package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.Routines.TwoBalls;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Util.RobotContainer;

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
            //new Taxi(robotContainer.getDriveTrain())
            //new OneBall(robotContainer.getDriveTrain(), lazySusanSubsystem, shooterSubsystem, firingPins)
            new TwoBalls(robotContainer.getDriveTrain(), lazySusanSubsystem, shooterSubsystem, firingPins, robotContainer.getIntake(), robotContainer.getField2d())
            //new ConditionalCommand(new WaitCommand(0), new ZeroTurnTable(lazySusanSubsystem), lazySusanSubsystem::getIsCal),
            
            );
        
    }
}
