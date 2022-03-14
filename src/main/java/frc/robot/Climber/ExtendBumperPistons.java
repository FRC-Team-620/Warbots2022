package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendBumperPistons extends CommandBase {
    ClimberSubsystem climberSubsystem;
    public ExtendBumperPistons(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        climberSubsystem.setBumperSolenoid(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
