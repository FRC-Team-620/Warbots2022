package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleHooks extends CommandBase {
    ClimberSubsystem climberSubsystem;
    protected int frames;

    public ToggleHooks(ClimberSubsystem climberSubsystem) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        this.frames = 0;
        System.out.println("Hooks were lowered");
        climberSubsystem.setHangingSolenoid(
            !climberSubsystem.getHangingSolenoid().get());    
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

