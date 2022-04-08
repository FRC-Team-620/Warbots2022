package frc.robot.Loader;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    protected CANSparkMax innerIntakeMotor;
    protected Solenoid intakeArmsSolenoid;
    protected CANSparkMax intakeArmsMotor;
    
    // protected boolean isClimbing;
    public Intake() {
        innerIntakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
        innerIntakeMotor.restoreFactoryDefaults();
        innerIntakeMotor.setInverted(true);

        innerIntakeMotor.setSmartCurrentLimit(35);
        
        intakeArmsSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
        intakeArmsSolenoid.set(false);

        intakeArmsMotor = new CANSparkMax(Constants.intakeArmsMotorID, MotorType.kBrushless);
        intakeArmsMotor.restoreFactoryDefaults();
        intakeArmsMotor.setInverted(true);

        intakeArmsMotor.setSmartCurrentLimit(35);        
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
    public void setInnerIntakeMotor(double speed) {
        this.innerIntakeMotor.set(speed);
    }

    public void extendIntakeArmsSolenoid() {
        intakeArmsSolenoid.set(true);
    }
    
    public void retractIntakeArmsSolenoid() {
        intakeArmsSolenoid.set(false);
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
