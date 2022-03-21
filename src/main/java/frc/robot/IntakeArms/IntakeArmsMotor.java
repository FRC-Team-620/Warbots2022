package frc.robot.IntakeArms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArmsMotor extends SubsystemBase {
    protected CANSparkMax intakeArmsMotor;
    // protected boolean isClimbing;

    public IntakeArmsMotor() {
        intakeArmsMotor = new CANSparkMax(Constants.intakeArmsMotorID, MotorType.kBrushless);
        intakeArmsMotor.restoreFactoryDefaults();
        intakeArmsMotor.setInverted(true);
        intakeArmsMotor.setSmartCurrentLimit(35);
    }

    public void enableIntakeArmsMotor() {
        intakeArmsMotor.set(-0.6);
    }

    public void disableIntakeArmsMotor() {
        intakeArmsMotor.set(0);
    }

    public void reverseIntakeArmsMotor() {
        intakeArmsMotor.set(0.6);
    }

    public double extendedIntakeArmsSpeed() {
        return intakeArmsMotor.get();
    }
}
