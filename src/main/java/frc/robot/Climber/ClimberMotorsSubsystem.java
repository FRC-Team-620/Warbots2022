package frc.robot.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberMotorsSubsystem extends SubsystemBase {
    protected final CANSparkMax leftClimberMotor, rightClimberMotor;
    protected final RelativeEncoder encoder;



    public ClimberMotorsSubsystem() {

        leftClimberMotor = new CANSparkMax(Constants.leftClimberMotorID, MotorType.kBrushless);
        rightClimberMotor = new CANSparkMax(Constants.rightClimberMotorID, MotorType.kBrushless);
        encoder = rightClimberMotor.getEncoder();

        leftClimberMotor.restoreFactoryDefaults();
        rightClimberMotor.restoreFactoryDefaults();

        IdleMode mode = IdleMode.kBrake; // brakes
        leftClimberMotor.setIdleMode(mode);
        rightClimberMotor.setIdleMode(mode);

        leftClimberMotor.follow(rightClimberMotor, true);

    }

    public void setWinchSpeed(double winchSpeed) {
        rightClimberMotor.set(winchSpeed);
    }

    public CANSparkMax getWinchMotor() {
        return rightClimberMotor;
    }
    public RelativeEncoder getWinchEncoder() {
        return encoder;
    }
    public double getWinchPosition() {
        return encoder.getPosition();
    }
    public double getWinchCountsPerRevolution() {
        return encoder.getCountsPerRevolution();
    }
}
