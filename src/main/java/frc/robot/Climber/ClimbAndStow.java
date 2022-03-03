package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.StowTurret;

public class ClimbAndStow extends SequentialCommandGroup {
    LazySusanSubsystem lazySusanSubsystem;
    ClimberMotorsSubsystem climberMotorsSubsystem;
    ShooterSubsystem shooterSubsystem;
    public ClimbAndStow(LazySusanSubsystem lazySusanSubsystem, ClimberMotorsSubsystem climberMotorsSubsystem, ShooterSubsystem shooterSubsystem) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(lazySusanSubsystem, climberMotorsSubsystem, shooterSubsystem);
        //shooterSubsystem.setDefaultCommand(new StowTurret(lazySusanSubsystem, shooterSubsystem));
        //CommandScheduler.getInstance().cancelAll();
        addCommands(
            new WinchExtend(climberMotorsSubsystem, Constants.winchMaxLimit)
        );
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
}
