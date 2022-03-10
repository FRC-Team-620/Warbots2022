package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WinchHold extends CommandBase {
    protected ClimberMotorsSubsystem climberMotorsSubsystem;
    protected double frames, holdPos;
    protected int endFrame;

    public WinchHold(ClimberMotorsSubsystem climberMotorsSubsystem, int endFrame) {
        addRequirements(climberMotorsSubsystem);
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.endFrame = endFrame;
    }

    @Override
    public void initialize() {
        this.frames = 0;
        System.out.println("WINCH HOLD WINCH HOLD");
        this.holdPos = this.climberMotorsSubsystem.getWinchPosition();
    }

    @Override
    public void execute() {
        double pos = this.climberMotorsSubsystem.getWinchPosition();
        double speed = Constants.diffConstWinchHold*(this.holdPos-pos);
        System.out.println("Speed: " + speed);
        this.climberMotorsSubsystem.setWinchSpeed(speed);
        frames++;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DONE DONE DONE");
        System.out.println("EXITING");
        this.climberMotorsSubsystem.setWinchSpeed(0);
    }

    @Override
    public boolean isFinished() {//climberSubsystem.getWinchMotor().getEncoder().getPosition() >= counts
        return this.frames >= this.endFrame;
    }
}
