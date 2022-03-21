package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Loader.Intake;

public class WinchExtend extends CommandBase {
    protected ClimberMotorsSubsystem climberMotorsSubsystem;
    protected Intake intake;
    protected double deltaCounts, targetCounts;

    public WinchExtend(ClimberMotorsSubsystem climberMotorsSubsystem, Intake intake, double deltaCounts) {
        addRequirements(climberMotorsSubsystem);
        this.climberMotorsSubsystem = climberMotorsSubsystem;
        this.intake = intake;
        this.deltaCounts = deltaCounts;
    }

    @Override
    public void initialize() {
        this.intake.extendIntakeArmsSolenoid();
        System.out.println("Current: " + this.climberMotorsSubsystem.getWinchPosition());
        this.targetCounts = this.climberMotorsSubsystem.getWinchPosition() + this.deltaCounts;
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