package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RaiseHooks extends CommandBase {
    ClimberSubsystem climberSubsystem;
    protected int frames;

    public RaiseHooks(ClimberSubsystem climberSubsystem) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        System.out.println("Hooks were raised");
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
