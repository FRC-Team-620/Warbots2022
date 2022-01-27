package frc.robot.Shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterCommand extends CommandBase {
    protected XboxController driverXbox;
    protected ShooterSubsystem shooterSubsystem;
    protected double bangBangTolerance = 0.05, intermittantSpeed = 0.3, minRPM = 1000, maxRPM = 3000;
    
    public ShooterCommand(ShooterSubsystem shooterSubsystem, XboxController driverXbox) {
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.driverXbox = driverXbox;
    }

    @Override
    public void execute() {
        double speed = driverXbox.getRightTriggerAxis();
        shooterSubsystem.setShooterSpeed(speed);
        // double rpm = driverXbox.getRightY() * (maxRPM - minRPM) + minRPM;
        // if (shooterSubsystem.getRPM() < rpm - bangBangTolerance * rpm)
        //     shooterSubsystem.setShooterSpeed(intermittantSpeed);
        // else
        //     shooterSubsystem.setShooterSpeed(0);

        // System.out.println(shooterSubsystem.getRPM());

        SmartDashboard.putNumber("Shooter RPM: ", shooterSubsystem.getRPM());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}