package frc.robot.Loader;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class LoaderSubsystem extends SubsystemBase {
    protected CANSparkMax loader;
    public LoaderSubsystem() {
        loader = new CANSparkMax(10, MotorType.kBrushless);
        loader.restoreFactoryDefaults();
        loader.setInverted(true);
    }
    public CANSparkMax getLoaderMotor() {
        return loader;
    }


}
