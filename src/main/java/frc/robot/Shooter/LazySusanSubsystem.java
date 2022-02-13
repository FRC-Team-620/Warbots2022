package frc.robot.Shooter;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class LazySusanSubsystem extends SubsystemBase {
    protected CANSparkMax lazySusan;
    protected RelativeEncoder encoder;
    public LazySusanSubsystem() {
        this.lazySusan = new CANSparkMax(9, MotorType.kBrushless);

        this.lazySusan.restoreFactoryDefaults();
        this.encoder = this.lazySusan.getEncoder();

        this.lazySusan.setIdleMode(IdleMode.kBrake);
    }
    
    public CANSparkMax getLazySusanMotor() {
        return this.lazySusan;
    }
    public RelativeEncoder getLazySusanEncoder() {
        return this.encoder;
    }
    public double getTicksPerMotorRotation() {
        return this.encoder.getCountsPerRevolution();
    }
}
