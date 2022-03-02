package frc.robot.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    // protected final CANSparkMax leftClimberMotor, rightClimberMotor;
    // protected final RelativeEncoder encoder;
    protected final Solenoid hangingSolenoid;
    protected final Solenoid armsSolenoid;



    public ClimberSubsystem() {

        // leftClimberMotor = new CANSparkMax(Constants.leftClimberMotorID, MotorType.kBrushless);
        // rightClimberMotor = new CANSparkMax(Constants.rightClimberMotorID, MotorType.kBrushless);
        // encoder = rightClimberMotor.getEncoder();

        // leftClimberMotor.restoreFactoryDefaults();
        // rightClimberMotor.restoreFactoryDefaults();

        // IdleMode mode = IdleMode.kBrake; // brakes
        // leftClimberMotor.setIdleMode(mode);
        // rightClimberMotor.setIdleMode(mode);

        // leftClimberMotor.follow(rightClimberMotor, true);

        hangingSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
        armsSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

        // leftShooterMotor.setInverted(false);
        // rightShooterMotor.setInverted(false);

    }

    // public void setWinchSpeed(double winchSpeed) {
    //     rightClimberMotor.set(winchSpeed);
    // }

    // public CANSparkMax getWinchMotor() {
    //     return rightClimberMotor;
    // }

    public Solenoid getHangingSolenoid() {
        return hangingSolenoid;
    }
    public Solenoid getArmsSolenoid() {
        return armsSolenoid;
    }

}
