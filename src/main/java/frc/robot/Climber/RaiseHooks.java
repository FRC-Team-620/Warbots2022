package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RaiseHooks extends CommandBase {
    ClimberSubsystem climberSubsystem;
    ClimberMotorsSubsystem climberMotorsSubsystem;
    protected int frames, maxFrame;

    public RaiseHooks(ClimberSubsystem climberSubsystem, ClimberMotorsSubsystem climberMotorsSubsystem) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        // this.maxFrame = maxFrame;
    }
    // public RaiseHooks(ClimberMotorsSubsystem climberMotorsSubsystem, ClimberSubsystem climberSubsystem) {
    //     this(climberMotorsSubsystem, climberSubsystem, 15);
    // }

    @Override
    public void initialize() {
        this.frames = 0;
        System.out.println("Hooks were raised");
        climberMotorsSubsystem.setWinchSpeed(0.0);
        climberSubsystem.setHangingSolenoid(false);
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
