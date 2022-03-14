package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class RaisePistons extends CommandBase {
    ClimberSubsystem climberSubsystem;
    protected double portion; // decimal percentage from 0 to 1
    protected int frames, targetFrames;

    public RaisePistons(ClimberSubsystem climberSubsystem) {
        this(climberSubsystem, 1.0);
    }

    public RaisePistons(ClimberSubsystem climberSubsystem, double portion) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
        this.portion = portion;
    }//hey hey hey everybody welcome back to the channel its myke cowmes make sure to smash that like button and suscribe to your mother

    @Override
    public void initialize() {
        this.frames = 0;
        System.out.println("Arms were raised");
        this.targetFrames = (int)Math.round(this.portion * Constants.pistonMaxFrames);
        climberSubsystem.setArmsSolenoid(true);
    }

    @Override
    public void execute() {
        this.frames++;
    }

    @Override
    public boolean isFinished() {
        return frames >= this.targetFrames;
    }

}
