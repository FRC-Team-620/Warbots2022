package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LowerArms extends CommandBase {
    ClimberSubsystem climberSubsystem;
    protected final int MAX_FRAMES = 15;
    protected double portion; // from 0 - 1
    protected int frames, targetFrames;

    public LowerArms(ClimberSubsystem climberSubsystem) {
        this(climberSubsystem, 1.0);
    }

    public LowerArms(ClimberSubsystem climberSubsystem, double portion) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
        this.portion = portion;
    }

    @Override
    public void initialize() {
        System.out.println("Arms were lowered");
        this.targetFrames = (int)(this.portion*MAX_FRAMES);
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
