package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    protected final CANSparkMax leftShooterMotor, rightShooterMotor;
    public ShooterSubsystem() {
        leftShooterMotor = new CANSparkMax(5, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(7, MotorType.kBrushless);

        leftShooterMotor.restoreFactoryDefaults();
        rightShooterMotor.restoreFactoryDefaults();

        IdleMode mode = IdleMode.kCoast; //brakes
        leftShooterMotor.setIdleMode(mode);
        rightShooterMotor.setIdleMode(mode);

        leftShooterMotor.follow(rightShooterMotor, true);
        //leftShooterMotor.setInverted(false);
        //rightShooterMotor.setInverted(false);

    }

    public void setShooterSpeed(double speed) {
        rightShooterMotor.set(speed);
    }
}