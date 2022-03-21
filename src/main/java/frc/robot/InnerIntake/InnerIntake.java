package frc.robot.InnerIntake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class InnerIntake extends SubsystemBase {
    protected CANSparkMax innerIntakeMotor;
    // protected boolean isClimbing;
    public InnerIntake() {
        innerIntakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
        innerIntakeMotor.restoreFactoryDefaults();
        innerIntakeMotor.setInverted(true);

        innerIntakeMotor.setSmartCurrentLimit(35);
    }
    public void enableInnerIntakeMotor() {
        innerIntakeMotor.set(1);
    }
    public void reverseInnerIntakeMotor() {
        innerIntakeMotor.set(-1);
    }
    public void disableInnerIntakeMotor() {
        innerIntakeMotor.set(0);
    }
}
