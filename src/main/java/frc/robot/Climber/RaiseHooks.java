package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RaiseHooks extends CommandBase {
    ClimberMotorsSubsystem climberMotorsSubsystem;
    ClimberSubsystem climberSubsystem;
    protected int frames;

    public RaiseHooks(ClimberMotorsSubsystem climberMotorsSubsystem, ClimberSubsystem climberSubsystem) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
        this.climberMotorsSubsystem = climberMotorsSubsystem;
    }

    @Override
    public void initialize() {
        System.out.println("Hooks were raised");
        climberMotorsSubsystem.setWinchSpeed(0.0);
        climberSubsystem.getHangingSolenoid().set(true);
    }

    @Override
    public void execute() {
        frames++;
    }

    @Override
    public void end(boolean interrupt) {
        this.climberMotorsSubsystem.setWinchSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return frames >= 15;
    }

}
