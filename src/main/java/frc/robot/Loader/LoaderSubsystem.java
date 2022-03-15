package frc.robot.Loader;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LoaderSubsystem extends SubsystemBase {
    protected CANSparkMax loader, extendedLoader;
    protected Solenoid trigger, extension;
    public LoaderSubsystem() {
        loader = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
        loader.restoreFactoryDefaults();
        loader.setInverted(true);

        // extendedLoader = new CANSparkMax(Constants.extendedIntakeMotorID, MotorType.kBrushless);
        // extendedLoader.restoreFactoryDefaults();
        // extendedLoader.setInverted(true);

        loader.setSmartCurrentLimit(25);

        trigger = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        trigger.set(false);
        extension = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
        extension.set(false);
    }
    public CANSparkMax getLoaderMotor() {
        return loader;
    }
    public CANSparkMax getExtendedLoaderMotor() {
        return extendedLoader;
    }
    public void enableExtendedLoaderMotor() {
        extendedLoader.set(1);
    }
    public void disableExtendedLoaderMotor() {
        extendedLoader.set(0);
    }
    public Solenoid getLoaderSolenoid() {
        return trigger;
    }
    public Solenoid getExtensionSolenoid() {
        return extension;
    }
}
