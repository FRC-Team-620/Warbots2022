package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FiringPins extends SubsystemBase {
    protected CANSparkMax loader, extendedLoader;
    protected Solenoid trigger, extension;
    // protected boolean isClimbing;
    public FiringPins() {
        trigger = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        trigger.set(false);
    }
    public void extendLoaderSolenoid() {
        trigger.set(true);
    }
    public void retractLoaderSolenoid() {
        trigger.set(false);
    }
}