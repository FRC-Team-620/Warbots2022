package frc.robot.Loader;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    protected CANSparkMax innerIntakeMotor;
    protected Solenoid intakeArmsVirtualSolenoidA;
    protected Solenoid intakeArmsVirtualSolenoidB;
    protected CANSparkMax intakeArmsMotor;
    protected DigitalInput intakeSwitch;
    
    // protected boolean isClimbing;
    public Intake() {
        innerIntakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
        innerIntakeMotor.restoreFactoryDefaults();
        innerIntakeMotor.setInverted(true);
        innerIntakeMotor.setIdleMode(IdleMode.kBrake);

        innerIntakeMotor.setSmartCurrentLimit(35);
        
        intakeArmsVirtualSolenoidA = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
        intakeArmsVirtualSolenoidB = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
        intakeArmsVirtualSolenoidA.set(false);
        intakeArmsVirtualSolenoidB.set(false);

        intakeArmsMotor = new CANSparkMax(Constants.intakeArmsMotorID, MotorType.kBrushless);
        intakeArmsMotor.restoreFactoryDefaults();
        intakeArmsMotor.setInverted(true);

        intakeArmsMotor.setSmartCurrentLimit(35);  
        
        intakeSwitch = new DigitalInput(Constants.intakeSwitchID);
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

    //implementing 836's idea
    public void extendIntakeArms() {
        intakeArmsVirtualSolenoidA.set(false);
        intakeArmsVirtualSolenoidB.set(false);
    }
    
    //implementing 836's idea
    public void retractIntakeArms() {
        intakeArmsVirtualSolenoidA.set(true);
        intakeArmsVirtualSolenoidB.set(true);    
    }

    //implementing 836's idea
    public void floatIntakeArms() {
        intakeArmsVirtualSolenoidA.set(false);
        intakeArmsVirtualSolenoidB.set(true);    
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

    public boolean getIntakeSwitch() {
        return intakeSwitch.get();
    }
}
