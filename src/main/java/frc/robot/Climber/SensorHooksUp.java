package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SensorHooksUp extends CommandBase {
    ClimberMotorsSubsystem climberMotorsSubsystem;
    ClimberSubsystem climberSubsystem;
    protected int frames, maxFrame;

    public SensorHooksUp(ClimberMotorsSubsystem climberMotorsSubsystem, ClimberSubsystem climberSubsystem) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
        this.climberMotorsSubsystem = climberMotorsSubsystem;
    }

    @Override
    public void initialize() {
        this.frames = 0;
        System.out.println("Hooks were raised");
        climberMotorsSubsystem.setWinchSpeed(0.0);
    }

    @Override
    public void execute() {
        //frames++;
        //System.out.println(climberMotorsSubsystem.getClimberSensor());
    }

    @Override
    public void end(boolean interrupt) {
        this.climberMotorsSubsystem.setWinchSpeed(0);
        climberSubsystem.setHangingSolenoid(false);
    }

    @Override
    public boolean isFinished() {
        
        return !(climberMotorsSubsystem.getClimberSensor());
    }

}
