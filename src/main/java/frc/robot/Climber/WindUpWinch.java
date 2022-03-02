package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WindUpWinch extends CommandBase {
    protected ClimberMotorsSubsystem climberMotorsSubsystem;
    protected double deltaCounts, targetCounts;

    public WindUpWinch(ClimberMotorsSubsystem climberMotorsSubsystem, double deltaCounts) {
        addRequirements(climberMotorsSubsystem);
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.deltaCounts = deltaCounts;
    }

    @Override
    public void initialize() {
        this.targetCounts = this.climberMotorsSubsystem.getWinchPosition() - this.deltaCounts;
        System.out.println("Winch begins wind up");
        this.climberMotorsSubsystem.setWinchSpeed(-1);
    }

    @Override
    public void execute() {
        System.out.println("Winch is winding");
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {//climberSubsystem.getWinchMotor().getEncoder().getPosition() <= -counts
        return climberMotorsSubsystem.getWinchPosition() <= Math.max(this.targetCounts, Constants.winchMinLimit);
    }
}
