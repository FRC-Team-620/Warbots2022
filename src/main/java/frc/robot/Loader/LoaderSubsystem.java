package frc.robot.Loader;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class LoaderSubsystem extends SubsystemBase {
    protected CANSparkMax loader;
    public LoaderSubsystem() {
        loader = new CANSparkMax(8, MotorType.kBrushless);

        loader.restoreFactoryDefaults();
    }
    public CANSparkMax getLoaderMotor() {
        return loader;
    }


}
