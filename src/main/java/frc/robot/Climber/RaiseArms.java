package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RaiseArms extends CommandBase {
    ClimberSubsystem climberSubsystem;
    protected int frames;

    public RaiseArms(ClimberSubsystem climberSubsystem) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        System.out.println("Arms were raised");
        climberSubsystem.getArmsSolenoid().set(true);
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
