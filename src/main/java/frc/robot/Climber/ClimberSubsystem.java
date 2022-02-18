package frc.robot.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    protected final CANSparkMax leftClimberMotor, rightClimberMotor;
    protected final RelativeEncoder encoder;

    public ClimberSubsystem() {
        leftClimberMotor = new CANSparkMax(Constants.leftClimberMotorID, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(Constants.rightClimberMotorID, MotorType.kBrushless);
        encoder = rightClimberMotor.getEncoder();

        leftClimberMotor.restoreFactoryDefaults();
        rightClimberMotor.restoreFactoryDefaults();

        IdleMode mode = IdleMode.kCoast; // brakes
        leftClimberMotor.setIdleMode(mode);
        rightClimberMotor.setIdleMode(mode);

        leftClimberMotor.follow(rightClimberMotor, true);
        // leftShooterMotor.setInverted(false);
        // rightShooterMotor.setInverted(false);

    }
}
