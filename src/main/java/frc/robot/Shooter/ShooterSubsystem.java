package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    protected final CANSparkMax leftShooterMotor, rightShooterMotor;
    public ShooterSubsystem() {
        leftShooterMotor = new CANSparkMax(5, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(6, MotorType.kBrushless);

        leftShooterMotor.restoreFactoryDefaults();
        rightShooterMotor.restoreFactoryDefaults();

        leftShooterMotor.follow(rightShooterMotor);
        leftShooterMotor.setInverted(true);

    }
}
