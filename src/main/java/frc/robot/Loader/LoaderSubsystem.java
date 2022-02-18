package frc.robot.Loader;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class LoaderSubsystem extends SubsystemBase {
    protected CANSparkMax loader;
    protected Solenoid trigger;
    public LoaderSubsystem() {
        loader = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
        loader.restoreFactoryDefaults();
        loader.setInverted(true);

        trigger = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        trigger.set(false);
    }
    public CANSparkMax getLoaderMotor() {
        return loader;
    }
    public Solenoid getSolenoid() {
        return trigger;
    }


}
