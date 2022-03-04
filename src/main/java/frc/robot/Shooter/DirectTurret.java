package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

public class DirectTurret extends CommandBase {

    protected LazySusanSubsystem lazySusanSubsystem;
    protected ShooterSubsystem shooterSubsystem;
    
    protected CANSparkMax lazySusanMotor;
    protected double tolerance = 0.1, targetPosition, setFinalEncoderCount;

    public DirectTurret(LazySusanSubsystem lazySusanSubsystem, ShooterSubsystem shooterSubsystem, 
        double targetPosition) {

        this.lazySusanSubsystem = lazySusanSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.targetPosition = targetPosition;
        this.setFinalEncoderCount = this.targetPosition;
        addRequirements(shooterSubsystem, lazySusanSubsystem);
        lazySusanMotor = this.lazySusanSubsystem.getLazySusanMotor();
    }
    public DirectTurret(LazySusanSubsystem lazySusanSubsystem, ShooterSubsystem shooterSubsystem,
        double targetPosition, double setFinalEncoderCount) {

        this(lazySusanSubsystem, shooterSubsystem, targetPosition);
        this.setFinalEncoderCount = setFinalEncoderCount;
    }

    @Override
    public void execute() {
        this.lazySusanMotor.set(
            Constants.diffConstLS*(this.targetPosition - this.lazySusanSubsystem.getLazySusanPosition()));
    }

    @Override
    public void end(boolean interrupt) {
        this.lazySusanMotor.getEncoder().setPosition(this.setFinalEncoderCount);
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