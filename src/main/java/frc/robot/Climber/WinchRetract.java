package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class WinchRetract extends CommandBase {
    protected ClimberMotorsSubsystem climberMotorsSubsystem;
    protected double deltaCounts, targetCounts;

    public WinchRetract(ClimberMotorsSubsystem climberMotorsSubsystem, double deltaCounts) {
        addRequirements(climberMotorsSubsystem);
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.deltaCounts = deltaCounts;
    }

    @Override
    public void initialize() {
        System.out.println("Current: " + this.climberMotorsSubsystem.getWinchPosition());
        this.targetCounts = this.climberMotorsSubsystem.getWinchPosition() - this.deltaCounts;
        System.out.println("Target: " + this.targetCounts);
        this.climberMotorsSubsystem.setWinchSpeed(-0.75);
        // System.out.println("Winch begins wind up");
    }

    @Override
    public void execute() {
        System.out.println("Winch is winding");
        // this.climberMotorsSubsystem.setWinchSpeed(
        //     Math.abs(this.targetCounts-this.climberMotorsSubsystem.getWinchPosition())>10?-1:-.2);
    }

    @Override
    public void end(boolean interrupted) {
        this.climberMotorsSubsystem.setWinchSpeed(0);
    }

    @Override
    public boolean isFinished() {//climberSubsystem.getWinchMotor().getEncoder().getPosition() <= -counts
        System.out.println("Pos: " + climberMotorsSubsystem.getWinchPosition());
        return climberMotorsSubsystem.getWinchPosition() <= 
            Math.max(this.targetCounts, Constants.winchMinLimit);
    }
}
