package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Loader.LoaderSubsystem;

public class ExtendArmsAndStow extends SequentialCommandGroup {
    ClimberMotorsSubsystem climberMotorsSubsystem;
    ClimberSubsystem climberSubsystem;
    LoaderSubsystem loaderSubsystem;

    public ExtendArmsAndStow(ClimberMotorsSubsystem climberMotorsSubsystem, 
    ClimberSubsystem climberSubsystem, LoaderSubsystem loaderSubsystem) {
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.climberSubsystem = climberSubsystem;
        this.loaderSubsystem = loaderSubsystem;
        addRequirements(climberMotorsSubsystem, climberSubsystem, loaderSubsystem);
        
        addCommands(
            new ExtendBumperPistons(this.climberSubsystem),
            new RaisePistons(this.climberSubsystem, 0.1), // Bump the arms up slightly
            new WinchExtend(this.climberMotorsSubsystem, this.loaderSubsystem, Constants.winchMaxLimit / 2.0),
            new LowerPistons(this.climberSubsystem, 0.1),
            new WinchExtend(this.climberMotorsSubsystem, this.loaderSubsystem, Constants.winchMaxLimit / 2.0)
        );
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
}