package frc.robot.Shooter;

import java.text.DecimalFormat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    protected final CANSparkMax leftShooterMotor, rightShooterMotor;
    protected final RelativeEncoder encoder;
    protected final DecimalFormat decFormat = new DecimalFormat("#.#");

    public ShooterSubsystem() {
        leftShooterMotor = new CANSparkMax(Constants.leftShooterMotorID, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(Constants.rightShooterMotorID, MotorType.kBrushless);
        encoder = rightShooterMotor.getEncoder();

        leftShooterMotor.restoreFactoryDefaults();
        rightShooterMotor.restoreFactoryDefaults();

        IdleMode mode = IdleMode.kCoast; // brakes
        leftShooterMotor.setIdleMode(mode);
        rightShooterMotor.setIdleMode(mode);

        leftShooterMotor.follow(rightShooterMotor, true);
        // leftShooterMotor.setInverted(false);
        // rightShooterMotor.setInverted(false);

    }

    public double getTicksPerMotorRotation() {
        return encoder.getCountsPerRevolution();
    }
    public double getRPM() {
        return Double.parseDouble(decFormat.format(encoder.getVelocity()));
    }
    public long getTotalWheelRotations() {
        return (long)encoder.getPosition(); // The conversion factor was previously set
    }
    // public double getRPM(long prevRotations, double secondsTimestep) {
    //     return (this.getTotalWheelRotations() - prevRotations) / (secondsTimestep/60);
    // }

    public void setShooterSpeed(double speed) {
        rightShooterMotor.set(speed);
    }
}