package frc.robot.Loader;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax innerIntakeMotor;
    private double intakeSpeedConstant = Constants.INTAKE_SPEED;

    // protected boolean isClimbing;
    public Intake() {
        innerIntakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
        innerIntakeMotor.restoreFactoryDefaults();
        innerIntakeMotor.setInverted(true);
        innerIntakeMotor.setIdleMode(IdleMode.kBrake);
        innerIntakeMotor.setSmartCurrentLimit(Constants.NEO550_CURRENT_LIMIT);
        SmartDashboard.putNumber("Intake/intakeSpeed", intakeSpeedConstant);
    }

    public void enableInnerIntakeMotor() {
        innerIntakeMotor.set(SmartDashboard.getNumber("Intake/intakeSpeed", intakeSpeedConstant));
    }

    public void disableInnerIntakeMotor() {
        innerIntakeMotor.set(0);
    }

    public void setInnerIntakeMotor(double speed) {
        this.innerIntakeMotor.set(speed);
    }
}
