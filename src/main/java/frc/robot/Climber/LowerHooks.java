package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LowerHooks extends CommandBase {
    ClimberSubsystem climberSubsystem;
    protected int frames;

    public LowerHooks(ClimberSubsystem climberSubsystem) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        this.frames = 0;
        System.out.println("Hooks were lowered");
        climberSubsystem.getHangingSolenoid().set(true);
    }

    @Override
    public void execute() {
        frames++;
    }

    @Override
    public boolean isFinished() {
        return frames >= 15;
    }

}

