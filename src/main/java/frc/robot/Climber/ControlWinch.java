package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class ControlWinch extends CommandBase {
    protected ClimberMotorsSubsystem climberMotorsSubsystem;
    protected double absoluteCounts, targetCounts;

    public ControlWinch(ClimberMotorsSubsystem climberMotorsSubsystem, double absoluteCounts) {
        addRequirements(climberMotorsSubsystem);
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.absoluteCounts = absoluteCounts;
    }

    @Override
    public void initialize() {
        System.out.println("Current: " + this.climberMotorsSubsystem.getWinchPosition());
        //this.targetCounts = this.climberMotorsSubsystem.getWinchPosition() + this.deltaCounts;
        // System.out.println("Winch begins wind down");
        System.out.println("Target: " + this.targetCounts);
        this.climberMotorsSubsystem.setWinchSpeed(0.75);
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
        System.out.println("Pos: " + this.climberMotorsSubsystem.getWinchPosition());
        boolean flag = climberMotorsSubsystem.getWinchPosition() >= 
            Math.min(this.targetCounts, Constants.winchMaxLimit);
        System.out.println("FLAG = " + flag);
        return flag;
    }
}