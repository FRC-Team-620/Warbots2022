package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class ExtendArmsAndStow extends SequentialCommandGroup {
    ClimberMotorsSubsystem climberMotorsSubsystem;
    ClimberSubsystem climberSubsystem;
    public ExtendArmsAndStow(ClimberMotorsSubsystem climberMotorsSubsystem, ClimberSubsystem climberSubsystem) {
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberMotorsSubsystem, climberSubsystem);
        
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
