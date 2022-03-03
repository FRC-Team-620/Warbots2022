package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class LowerPistons extends CommandBase {
    ClimberSubsystem climberSubsystem;
    protected double portion; // decimal percentage from 0 to 1
    protected int frames, targetFrames;

    public LowerPistons(ClimberSubsystem climberSubsystem) {
        this(climberSubsystem, 1.0);
    }

    public LowerPistons(ClimberSubsystem climberSubsystem, double portion) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
        this.portion = portion;
    }

    @Override
    public void initialize() {
        System.out.println("Arms were lowered");
        this.targetFrames = (int)Math.round(this.portion * Constants.pistonMaxFrames);
        climberSubsystem.getArmsSolenoid().set(false);
    }

    @Override
    public void execute() {
        this.frames++;
    }

    @Override
    public boolean isFinished() {
        return this.frames >= this.targetFrames;
    }

}
