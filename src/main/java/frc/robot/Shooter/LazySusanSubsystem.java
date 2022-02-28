package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class LazySusanSubsystem extends SubsystemBase {
    protected SimableCANSparkMax lazySusan;
    protected RelativeEncoder encoder;

    // Sim
    private FlywheelSim simlazySusan;
    private RevEncoderSimWrapper simEncoder;
    public Rotation2d turrentRotation;
    public LazySusanSubsystem() {
        lazySusan = new SimableCANSparkMax(Constants.lazySusanID, MotorType.kBrushless);

        lazySusan.restoreFactoryDefaults();
        encoder = lazySusan.getEncoder();
        IdleMode mode = IdleMode.kBrake; // brakes
        lazySusan.setIdleMode(mode);

        turrentRotation = new Rotation2d();
        if (RobotBase.isSimulation()) {
            initSim();
        }
    }

    private void initSim() {
        simlazySusan = new FlywheelSim(DCMotor.getNeo550(1), (156/16), Constants.kSimTurntableInertia); //TODO: add gear ratio
        simEncoder = RevEncoderSimWrapper.create(this.lazySusan);
    }

    public CANSparkMax getLazySusanMotor() {
        return lazySusan;
    }

    public RelativeEncoder getLazySusanEncoder() {
        return encoder;
    }

    public double getTicksPerMotorRotation() {
        return encoder.getCountsPerRevolution();
    }
    @Override
    public void periodic() {
        turrentRotation = Rotation2d.fromDegrees(encoder.getPosition());
    }
    @Override
    public void simulationPeriodic() {
        simlazySusan.setInputVoltage(lazySusan.get() * RobotController.getInputVoltage());
        simlazySusan.update(Constants.kSimUpdateTime);
        simEncoder.setVelocity(simlazySusan.getAngularVelocityRPM());
        // simlazySusan.
        simEncoder.setDistance(simEncoder.getPosition()+(simlazySusan.getAngularVelocityRPM()*Constants.kSimUpdateTime)); //TODO: Hacky should be fixed/use proper linear system (square raymond sum)
        SmartDashboard.putNumber("Turntable Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Turntable ticks", encoder.getPosition());
        SmartDashboard.putNumber("Turntable Set", lazySusan.get());
    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return this.simlazySusan.getCurrentDrawAmps();
        }
        return this.lazySusan.getOutputCurrent();
    }
}
