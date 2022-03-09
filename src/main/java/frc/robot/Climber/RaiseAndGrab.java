package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class RaiseAndGrab extends SequentialCommandGroup {
    ClimberSubsystem climberSubsystem;
    ClimberMotorsSubsystem climberMotorsSubsystem;
    int raiseHooksFrames = 15;
    public RaiseAndGrab(ClimberMotorsSubsystem climberMotorsSubsystem, ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        addRequirements(this.climberMotorsSubsystem, this.climberSubsystem);
        addCommands(
            new WinchRetract(climberMotorsSubsystem, Constants.winchMaxLimit/2.0),
            new LowerHooks(climberSubsystem),
            new LowerPistons(climberSubsystem),
            new WinchRetract(climberMotorsSubsystem, Constants.winchMaxLimit),
            // new RaiseHooks(climberMotorsSubsystem, climberSubsystem);
            new ParallelCommandGroup(
                new RaiseHooks(climberMotorsSubsystem, climberSubsystem, this.raiseHooksFrames),
                new WinchHold(climberMotorsSubsystem, this.raiseHooksFrames))
        );
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
}
