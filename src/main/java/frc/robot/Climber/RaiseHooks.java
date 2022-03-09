package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RaiseHooks extends CommandBase {
    ClimberMotorsSubsystem climberMotorsSubsystem;
    ClimberSubsystem climberSubsystem;
    protected int frames, maxFrame;

    public RaiseHooks(ClimberMotorsSubsystem climberMotorsSubsystem, ClimberSubsystem climberSubsystem, int maxFrame) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.maxFrame = maxFrame;
    }
    public RaiseHooks(ClimberMotorsSubsystem climberMotorsSubsystem, ClimberSubsystem climberSubsystem) {
        this(climberMotorsSubsystem, climberSubsystem, 15);
    }

    @Override
    public void initialize() {
        this.frames = 0;
        System.out.println("Hooks were raised");
        climberMotorsSubsystem.setWinchSpeed(0.0);
        climberSubsystem.getHangingSolenoid().set(false);
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
        return this.frames >= this.maxFrame;
    }

}
