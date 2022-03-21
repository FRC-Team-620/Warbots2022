package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Loader.Intake;

public class ExtendArmsAndStow extends SequentialCommandGroup {
    ClimberMotorsSubsystem climberMotorsSubsystem;
    ClimberSubsystem climberSubsystem;
    Intake intake;

    public ExtendArmsAndStow(ClimberMotorsSubsystem climberMotorsSubsystem, 
    ClimberSubsystem climberSubsystem, Intake intake) {
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.climberSubsystem = climberSubsystem;
        this.intake = intake;
        addRequirements(climberMotorsSubsystem, climberSubsystem, intake);
        // loaderSubsystem.setIsClimbing(true);
        
        addCommands(
            new ExtendBumperPistons(this.climberSubsystem),
            new RaisePistons(this.climberSubsystem, 0.1), // Bump the arms up slightly
            new WinchExtend(this.climberMotorsSubsystem, this.intake, Constants.winchMaxLimit / 2.0),
            new LowerPistons(this.climberSubsystem, 0.1),
            new WinchExtend(this.climberMotorsSubsystem, this.intake, Constants.winchMaxLimit / 2.0)
        );
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
}