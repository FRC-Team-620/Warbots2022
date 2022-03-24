package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class LazySusanSubsystem extends SubsystemBase {
    protected SimableCANSparkMax lazySusan;
    protected RelativeEncoder encoder;
    private PIDController lazySusanPID;
    private final double kP = 0.0004, kI = 0, kD = 0;
    public final double lowLimit = 0, highLimit = 20;
    // private double turntableThresh = 35;

    public LazySusanSubsystem() {
        lazySusan = new SimableCANSparkMax(Constants.lazySusanID, MotorType.kBrushless);

        lazySusan.restoreFactoryDefaults();
        encoder = lazySusan.getEncoder();
        IdleMode mode = IdleMode.kBrake; // brakes
        lazySusan.setIdleMode(mode);

        lazySusan.setSmartCurrentLimit(35);

        lazySusanPID = new PIDController(kP, kI, kD);
        lazySusanPID.setTolerance(10);
        lazySusanPID.setIntegratorRange(-10, 10);

        SmartDashboard.putData(lazySusanPID);
    }

    @Override
    public void periodic() {
        lazySusanPID.calculate(lazySusan.getEncoder().getPosition());

        double lazySusanOutput = lazySusanPID.calculate(lazySusan.getEncoder().getPosition());

        lazySusan.set(MathUtil.clamp(lazySusanOutput, -1, 1) * 0.2);

        SmartDashboard.putNumber("TurretPos", lazySusan.getEncoder().getPosition());
    }

    public void setTurretPosition(double x) {
        lazySusanPID.setSetpoint(MathUtil.clamp(x, lowLimit, highLimit));
    }

    public boolean atTurretPosition() {
        return lazySusanPID.atSetpoint();
    }



    public CANSparkMax getLazySusanMotor() {
        return lazySusan;
    }

    public void setLazySusanSpeed(double speed) {
        //lazySusan.set(speed);
    }
    
    public RelativeEncoder getLazySusanEncoder() {
        return encoder;
    }

    public double getLazySusanPosition() {
        return encoder.getPosition();
    }

    public void setLazySusanPosition(double p) {
        //encoder.setPosition(p);
    }

    public double getTicksPerMotorRotation() {
        return encoder.getCountsPerRevolution();
    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return this.simlazySusan.getCurrentDrawAmps();
        }
        return this.lazySusan.getOutputCurrent();
    }

    /**
     * Simulation Code
     */
    private DCMotorSim simlazySusan;
    private RevEncoderSimWrapper simEncoder;
    public Rotation2d simTurrentRotation;
    private boolean simInit = false;

    private void initSim() {
        simTurrentRotation = new Rotation2d();
        simlazySusan = new DCMotorSim(DCMotor.getNeo550(1), Constants.kSimTurntableGearRatio,
                Constants.kSimTurntableInertia); // TODO: add gear ratio
        simEncoder = RevEncoderSimWrapper.create(this.lazySusan);
    }

    @Override
    public void simulationPeriodic() {
        if (!simInit) {
            initSim();
            simInit = true;
        }
        simTurrentRotation = Rotation2d.fromDegrees((encoder.getPosition() / Constants.kSimTurntableGearRatio) * 360);
        simlazySusan.setInputVoltage(lazySusan.get() * RobotController.getInputVoltage());
        simlazySusan.update(Constants.kSimUpdateTime);
        simEncoder.setVelocity(simlazySusan.getAngularVelocityRPM());
        simEncoder.setDistance(simlazySusan.getAngularPositionRotations() * Constants.kSimTurntableGearRatio / 5);
        // TODO: Remove magic number 5 that represents the first gear reduction
        SmartDashboard.putNumber("Turntable Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Turntable ticks", encoder.getPosition());
        SmartDashboard.putNumber("Turntable Set", lazySusan.get());
    }
}
