package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class DirectTurret extends CommandBase {

    protected LazySusanSubsystem lazySusanSubsystem;
    protected ShooterSubsystem shooterSubsystem;
    
    protected CANSparkMax lazySusanMotor;
    protected double tolerance = 0.07, speed = 0.4, targetPosition, setFinalEncoderCount;
    protected boolean directionToTarget; // true is counterclockwise

    public DirectTurret(LazySusanSubsystem lazySusanSubsystem, 
        ShooterSubsystem shooterSubsystem, double targetPosition) {

        this.lazySusanSubsystem = lazySusanSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.targetPosition = targetPosition;
        this.directionToTarget = targetPosition > lazySusanSubsystem.getLazySusanPosition();
        this.setFinalEncoderCount = this.targetPosition;
        addRequirements(shooterSubsystem, lazySusanSubsystem);
        lazySusanMotor = this.lazySusanSubsystem.getLazySusanMotor();
    }
    public DirectTurret(LazySusanSubsystem lazySusanSubsystem, 
        ShooterSubsystem shooterSubsystem, double targetPosition, 
        double setFinalEncoderCount) {

        this(lazySusanSubsystem, shooterSubsystem, targetPosition);
        this.setFinalEncoderCount = setFinalEncoderCount;
    }

    @Override
    public void initialize() {
        this.lazySusanMotor.setOpenLoopRampRate(0.2);
        lazySusanMotor.set((this.directionToTarget?1:-1)*this.speed);
    }

    @Override
    public void execute() {
        boolean targetDir = this.targetPosition > this.lazySusanSubsystem.getLazySusanPosition();
        if(targetDir != this.directionToTarget)
            lazySusanMotor.set((targetDir?1:-1)*this.speed);
        this.directionToTarget = targetDir;
        
        // if(this.targetPosition > this.lazySusanSubsystem.getLazySusanPosition())
        //     lazySusanMotor.set(this.speed);
        // else
        //     lazySusanMotor.set(-this.speed);

        // this.lazySusanMotor.set(
        //     Constants.diffConstLS*(this.targetPosition - this.lazySusanSubsystem.getLazySusanPosition()));
    }

    @Override
    public void end(boolean interrupt) {
        System.out.println("HERE!!!");
        this.lazySusanMotor.getEncoder().setPosition(this.setFinalEncoderCount);
        this.lazySusanMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        double p = this.lazySusanSubsystem.getLazySusanPosition();
        System.out.println(p);
        // System.out.println("LS POS: " + p);
        // System.out.println("MAX: " + this.targetPosition*(1+this.tolerance));
        // System.out.println("MIN: " + this.targetPosition*(1-this.tolerance));
        return (this.targetPosition*(1+this.tolerance) >= p && 
            p >= this.targetPosition*(1-this.tolerance)) || 
            (this.targetPosition*(1+this.tolerance) <= p && 
            p <= this.targetPosition*(1-this.tolerance));
    }
}