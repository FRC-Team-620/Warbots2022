package frc.robot.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class SpinUpMotors extends CommandBase{
    protected ShooterSubsystem shooterSubsystem;
    protected double acceleration;
    public SpinUpMotors(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        acceleration = 0;
    }
    @Override
    public void execute() {
        acceleration = Constants.diffConstShooter * (shooterSubsystem.getTargetRPM() - shooterSubsystem.getRPM());
        SmartDashboard.putNumber("Acceleration: ", acceleration);
        // if(Math.abs(rpm-currRPM) > (bangBangTolerance * rpm))
        shooterSubsystem.setShooterSpeedAndUpdate(shooterSubsystem.getCurrentSpeed() + acceleration);
        // System.out.println(rpm);
        if(shooterSubsystem.getTargetRPM() == 0.0) {
            shooterSubsystem.setShooterSpeedAndUpdate(0);
        }
        SmartDashboard.putNumber("Shooter RPM: ",
                ShooterMath.roundUpToNearestMultiple(shooterSubsystem.getRPM(), 5));
    }
}
