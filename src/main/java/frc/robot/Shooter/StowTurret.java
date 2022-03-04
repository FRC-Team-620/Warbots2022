package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

public class StowTurret extends CommandBase {

    protected LazySusanSubsystem lazySusanSubsystem;
    protected ShooterSubsystem shooterSubsystem;
    
    protected CANSparkMax lazySusanMotor;
    protected double stowedPosition = 60, tolerance = 0.1;
    protected double frames = 0;

    public StowTurret(LazySusanSubsystem lazySusanSubsystem, ShooterSubsystem shooterSubsystem) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem, lazySusanSubsystem);
        lazySusanMotor = lazySusanSubsystem.getLazySusanMotor();
    }

    @Override
    public void execute() {
        lazySusanMotor.set(
            Constants.diffConstLS*(stowedPosition - lazySusanSubsystem.getLazySusanPosition()));
    }

    @Override
    public boolean isFinished() {
        double p = lazySusanSubsystem.getLazySusanPosition();
        System.out.println("LS POS: " + p);
        System.out.println("MAX: " + this.stowedPosition*(1+this.tolerance));
        System.out.println("MIN: " + this.stowedPosition*(1-this.tolerance));
        return this.stowedPosition*(1+this.tolerance) >= p && 
            p >= this.stowedPosition*(1-this.tolerance);
    }
}