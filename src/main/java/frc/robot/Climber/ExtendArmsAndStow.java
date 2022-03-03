package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.ShooterCommand;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.StowTurret;

public class ExtendArmsAndStow extends SequentialCommandGroup {
    LazySusanSubsystem lazySusanSubsystem;
    ClimberMotorsSubsystem climberMotorsSubsystem;
    ClimberSubsystem climberSubsystem;
    ShooterSubsystem shooterSubsystem;
    ShooterCommand shooterCommand;
    public ExtendArmsAndStow(LazySusanSubsystem lazySusanSubsystem, ClimberMotorsSubsystem climberMotorsSubsystem, ClimberSubsystem climberSubsystem, ShooterSubsystem shooterSubsystem, ShooterCommand shooterCommand) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.climberSubsystem = climberSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.shooterCommand = shooterCommand;
        addRequirements(lazySusanSubsystem, climberMotorsSubsystem, shooterSubsystem);
        shooterSubsystem.setDefaultCommand(new StowTurret(lazySusanSubsystem, shooterSubsystem));
        
        //CommandScheduler.getInstance().cancelAll();
        addCommands(
            new RaisePistons(this.climberSubsystem, 0.1), // Bump the arms up slightly
            new WinchExtend(this.climberMotorsSubsystem, Constants.winchMaxLimit / 2.0),
            new LowerPistons(this.climberSubsystem, 0.1),
            new WinchExtend(this.climberMotorsSubsystem, Constants.winchMaxLimit / 2.0)
        );
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
}
