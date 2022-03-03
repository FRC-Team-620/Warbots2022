package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

public class StowTurret extends CommandBase {

    protected LazySusanSubsystem lazySusanSubsystem;
    protected ShooterSubsystem shooterSubsystem;
    
    protected CANSparkMax lazySusanMotor;
    protected double stowedPosition = 50;

    public StowTurret(LazySusanSubsystem lazySusanSubsystem, ShooterSubsystem shooterSubsystem) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        lazySusanMotor = lazySusanSubsystem.getLazySusanMotor();
    }

    @Override
    public void execute() {
        System.out.println("(Q&&#^$*&Q#^$&");
        lazySusanMotor.set(
            Constants.diffConstLS*(stowedPosition - lazySusanSubsystem.getLazySusanPosition()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}