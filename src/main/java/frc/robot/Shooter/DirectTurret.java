package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

public class DirectTurret extends CommandBase {

    protected LazySusanSubsystem lazySusanSubsystem;
    protected ShooterSubsystem shooterSubsystem;
    
    protected CANSparkMax lazySusanMotor;
    protected double tolerance = 0.1, targetPosition;
    protected boolean resetUponCompletion;

    public DirectTurret(LazySusanSubsystem lazySusanSubsystem, ShooterSubsystem shooterSubsystem,
        double targetPosition, boolean resetUponCompletion) {

        this.lazySusanSubsystem = lazySusanSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.targetPosition = targetPosition;
        this.resetUponCompletion = resetUponCompletion;
        addRequirements(shooterSubsystem, lazySusanSubsystem);
        lazySusanMotor = this.lazySusanSubsystem.getLazySusanMotor();
    }

    @Override
    public void execute() {
        this.lazySusanMotor.set(
            Constants.diffConstLS*(this.targetPosition - this.lazySusanSubsystem.getLazySusanPosition()));
    }

    @Override
    public void end(boolean interrupt) {
        if(this.resetUponCompletion)
            this.lazySusanMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        double p = this.lazySusanSubsystem.getLazySusanPosition();
        // System.out.println("LS POS: " + p);
        // System.out.println("MAX: " + this.targetPosition*(1+this.tolerance));
        // System.out.println("MIN: " + this.targetPosition*(1-this.tolerance));
        return this.targetPosition*(1+this.tolerance) >= p && 
            p >= this.targetPosition*(1-this.tolerance);
    }
}