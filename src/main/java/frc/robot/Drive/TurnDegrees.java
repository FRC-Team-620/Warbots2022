package frc.robot.Drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class TurnDegrees extends CommandBase {
    Drivetrain drivetrain;
    AHRS gyro;
    double degrees, targetDegrees;
    
    public TurnDegrees(Drivetrain drivetrain, double degrees) {
        this.drivetrain = drivetrain;
        this.gyro = new AHRS(SerialPort.Port.kMXP);
    }

    public void initialize() {
        this.targetDegrees = this.gyro.getRotation2d().getDegrees() + this.degrees;
    }

    public void execute() {
        double speed = Constants.diffConstTurn*(this.targetDegrees-this.gyro.getRotation2d().getDegrees());
        drivetrain.tankDriveSet(speed, -speed);
    }
}
