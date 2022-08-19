package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class RaiseAndGrab extends SequentialCommandGroup {
    ClimberSubsystem climberSubsystem;
    ClimberMotorsSubsystem climberMotorsSubsystem;
    int raiseHooksFrames = 100;
    public RaiseAndGrab(ClimberMotorsSubsystem climberMotorsSubsystem, ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        addRequirements(this.climberMotorsSubsystem, this.climberSubsystem);
        addCommands(
            new ParallelCommandGroup(
                new WinchRetract(climberMotorsSubsystem, Constants.winchMaxLimit/2.0),
                new LowerPistons(climberSubsystem)),
            // new ParallelCommandGroup(
            //     new LowerHooks(climberSubsystem),
            //     new WinchRetract(climberMotorsSubsystem, Constants.winchMaxLimit)
            // ),
            new LowerHooks(climberSubsystem),
            //new WinchRetract(climberMotorsSubsystem, Constants.winchMaxLimit),
            new SensorWinchRetract(climberMotorsSubsystem),
            // new RaiseHooks(climberMotorsSubsystem, climberSubsystem);
            new ParallelCommandGroup(
                new RaiseHooks(climberSubsystem, climberMotorsSubsystem),
                //new SensorHooksUp(climberMotorsSubsystem, climberSubsystem),
                new WinchHold(climberMotorsSubsystem, -5, this.raiseHooksFrames))
        );
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
}
