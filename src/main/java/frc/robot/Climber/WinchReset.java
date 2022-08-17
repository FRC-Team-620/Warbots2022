package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class WinchReset extends CommandBase {
    protected ClimberMotorsSubsystem climberMotorsSubsystem;

    public WinchReset(ClimberMotorsSubsystem climberMotorsSubsystem) {
        addRequirements(climberMotorsSubsystem);
        this.climberMotorsSubsystem = climberMotorsSubsystem;
    }

    @Override
    public void initialize() {
        this.climberMotorsSubsystem.setWinchSpeed(-0.75);
    }

    @Override
    public void execute() {
        System.out.println("Winch is winding");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("EXITING");
        this.climberMotorsSubsystem.setWinchSpeed(0);
    }

    @Override
    public boolean isFinished() {//climberSubsystem.getWinchMotor().getEncoder().getPosition() >= counts
        return this.climberMotorsSubsystem.getWinchPosition() <= Constants.winchMinLimit;
    }
}
