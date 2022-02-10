package frc.robot.Shooter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class LazySusanSubsystem extends SubsystemBase {
    protected CANSparkMax lazySusan;
    public LazySusanSubsystem() {
        lazySusan = new CANSparkMax(9, MotorType.kBrushless);

        lazySusan.restoreFactoryDefaults();

        IdleMode mode = IdleMode.kBrake; //brakes
        lazySusan.setIdleMode(mode);
    }
    
    public CANSparkMax getLazySusanMotor() {
        return lazySusan;
    }
}
