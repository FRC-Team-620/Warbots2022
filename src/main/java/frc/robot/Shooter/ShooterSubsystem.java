package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    protected final CANSparkMax leftShooterMotor, rightShooterMotor;
    protected final RelativeEncoder encoder;

    public ShooterSubsystem() {
        leftShooterMotor = new CANSparkMax(5, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(7, MotorType.kBrushless);
        encoder = rightShooterMotor.getEncoder();

        leftShooterMotor.restoreFactoryDefaults();
        rightShooterMotor.restoreFactoryDefaults();

        IdleMode mode = IdleMode.kCoast; // brakes
        leftShooterMotor.setIdleMode(mode);
        rightShooterMotor.setIdleMode(mode);

        leftShooterMotor.follow(rightShooterMotor, true);
        // leftShooterMotor.setInverted(false);
        // rightShooterMotor.setInverted(false);

    }

    public double getRPM() {
        return encoder.getPosition() / encoder.getCountsPerRevolution() * Constants.shooterGearRatio;
    }

    public void setShooterSpeed(double speed) {
        rightShooterMotor.set(speed);
    }
}