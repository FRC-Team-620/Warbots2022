package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LowerArms extends CommandBase {
    ClimberSubsystem climberSubsystem;
    protected int frames;

    public LowerArms(ClimberSubsystem climberSubsystem) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        System.out.println("Arms were lowered");
        climberSubsystem.getArmsSolenoid().set(false);
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
