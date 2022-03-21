package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FiringPins extends SubsystemBase {
    protected CANSparkMax loader, extendedLoader;
    protected Solenoid firingPinsSolenoid, extension;
    // protected boolean isClimbing;
    public FiringPins() {
        firingPinsSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        firingPinsSolenoid.set(false);
    }
    public void extendFiringPinsSolenoid() {
        firingPinsSolenoid.set(true);
    }
    public void retractFiringPinsSolenoid() {
        firingPinsSolenoid.set(false);
    }
}