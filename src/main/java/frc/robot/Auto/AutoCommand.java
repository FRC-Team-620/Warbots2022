package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auto.Routines.OneBall;
import frc.robot.Auto.Routines.Taxi;
import frc.robot.Auto.Routines.TwoBalls;
import frc.robot.Drive.DriveForwardsEncoder;
import frc.robot.Drive.Drivetrain;
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
            //new Taxi(robotContainer.getDriveTrain())
            //new OneBall(robotContainer.getDriveTrain(), lazySusanSubsystem, shooterSubsystem, firingPins)
            new TwoBalls(robotContainer.getDriveTrain(), lazySusanSubsystem, shooterSubsystem, firingPins, robotContainer.getIntake())
            //new ConditionalCommand(new WaitCommand(0), new ZeroTurnTable(lazySusanSubsystem), lazySusanSubsystem::getIsCal),
            
            );
        
    }
}
