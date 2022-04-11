package frc.robot.Shooter;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class LazySusanSubsystem extends SubsystemBase {
    private final SimableCANSparkMax motor;
    private final RelativeEncoder encoder;
    private PIDController lazySusanPID;
    private Rotation2d turretRotation;
    private Rotation2d desiredRotation;
    private final Supplier<Pose2d> robotBasePose;
    private boolean isGyroLocking;
    private final double countsToDegreesFactor = (1.0 / 25.0) * (20.0 / 156.0) * 360.0;
    private double modSpeed = 1;
    private final double kP = 0.000060, kI = 0.000000, kD = 0;// KI0.00004 kP = 0.060000, kI = 0.003000,  TODO: Tune PID Loop
    private boolean isCal;
    //private boolean isDisabled;
    public DigitalInput calSwitch;
    public final double lowLimitDegrees = -175.0;
    public final double highLimitDegrees = 185.0;
    // Left 45.690002, Right -45.356651, AbsoluteMaxRange 90
    // private double turntableThresh = 35;

    public LazySusanSubsystem(Supplier<Pose2d> robotBasePose) {
        this.isGyroLocking = false;
        this.robotBasePose = robotBasePose;
        this.motor = new SimableCANSparkMax(Constants.lazySusanID, MotorType.kBrushless);
        this.calSwitch = new DigitalInput(Constants.calSwitchID);
        this.motor.restoreFactoryDefaults();
        this.encoder = this.motor.getEncoder();
        encoder.setPositionConversionFactor(countsToDegreesFactor);
        this.encoder.setPosition(Constants.stowedDegrees);
        IdleMode mode = IdleMode.kBrake; // brakes
        this.motor.setIdleMode(mode);
        this.motor.setSmartCurrentLimit(18);
        this.isGyroLocking = false;

        // lazySusan.setInverted(true);
        lazySusanPID = new PIDController(kP, kI, kD);
        lazySusanPID.setTolerance(1);
        // lazySusanPID.setIntegratorRange(-10, 10);
        
        turretRotation = desiredRotation = Rotation2d.fromDegrees(Constants.stowedDegrees);
        isCal = false;
        //isDisabled =
        SmartDashboard.putData("Turret/PIDController[12]", lazySusanPID);
    }

    @Override
    public void periodic() {
        this.turretRotation = Rotation2d.fromDegrees(encoder.getPosition());
        double degrees = isGyroLocking ? desiredRotation.minus(robotBasePose.get().getRotation()).getDegrees() : desiredRotation.getDegrees();

        lazySusanPID.setSetpoint(MathUtil.clamp(degrees, lowLimitDegrees, highLimitDegrees));
        // System.out.println("highLimit " + degrees + " " + encoder.getPosition() + " " + highLimitCounts);
        double pidOutput = MathUtil.clamp(lazySusanPID.calculate(encoder.getPosition()), -1, 1);

        if (encoder.getPosition() > highLimitDegrees + 5) { // + 5 / countsToDegreesFactor
            pidOutput = MathUtil.clamp(pidOutput, -1, 0);
        }

        if (encoder.getPosition() < lowLimitDegrees - 5) {
            pidOutput = MathUtil.clamp(pidOutput, 0, 1);
        }

        motor.set(pidOutput * modSpeed);

        SmartDashboard.putNumber("Turret/Raw Encoder", encoder.getPosition());
        SmartDashboard.putNumber("Turret/Motor Percentage", motor.get());
        SmartDashboard.putBoolean("Turret/Is Calibrated", isCal);
        SmartDashboard.putBoolean("Turret/Calibration Switch", islimitSwitchPressed());
        SmartDashboard.putNumber("Turret/Desired Rotation", desiredRotation.getDegrees());
        SmartDashboard.putNumber("Turret/Rotation Degrees", turretRotation.getDegrees());
        SmartDashboard.putBoolean("Turret/Is Gyro Locking", isGyroLocking);
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public double getModSpeed() {
        return modSpeed;
    }

    public void setModSpeed(double modSpeed) {
        this.modSpeed = modSpeed;
    }

    public boolean getIsGyroLocking() {
        return isGyroLocking;
    }

    public void setIsGyroLocking(boolean isGyroLocking) {
        // TODO: Fix gyro locking problems (turning randomly?)
        this.isGyroLocking = isGyroLocking;
    }

    public boolean getIsCal() {
        return isCal;
    }
    
    public boolean getCalSensorState() {
        return calSwitch.get();
    }

    public void setIsCal(boolean isCal) {
        this.isCal = isCal;
    }

    public Rotation2d getRotation() {
        return turretRotation;
    }

    public Rotation2d getDesiredDegrees() {
        return desiredRotation;
    }

    public double getRawSetpoint() {
        return lazySusanPID.getSetpoint();
    }

    public void setTurretPositionDegrees(Rotation2d rot) {
        desiredRotation = rot;
        // this.setTurretPosition(degrees/countToDegreesFactor);
    }

    public boolean atTurretPosition() {
        return lazySusanPID.atSetpoint();
    }

    public void setEncoderPosition(double p) {
        encoder.setPosition(p);
        stop();
    }

    public void stop() {
        lazySusanPID.reset();
        lazySusanPID.setSetpoint(encoder.getPosition());
    }

    public void setHomePosition() {
        setIsGyroLocking(false);
        double limitPos = 38.8;//-45
        setEncoderPosition(limitPos);
        stop();
        setIsCal(true);
    }

    public boolean islimitSwitchPressed(){
        return !calSwitch.get();
    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return this.simlazySusan.getCurrentDrawAmps();
        }
        return this.motor.getOutputCurrent();
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
        simEncoder = RevEncoderSimWrapper.create(this.motor);
    }

    @Override
    public void simulationPeriodic() {
        if (!simInit) {
            initSim();
            simInit = true;
        }
        simTurrentRotation = turretRotation;
        simlazySusan.setInputVoltage(motor.get() * RobotController.getInputVoltage());
        simlazySusan.update(Constants.kSimUpdateTime);
        simEncoder.setVelocity(simlazySusan.getAngularVelocityRPM());
        simEncoder.setDistance(simlazySusan.getAngularPositionRotations() * 180);
        // TODO: Remove magic number 5 that represents the first gear reduction
        SmartDashboard.putNumber("Turret/Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Turret/ticks", encoder.getPosition());
        SmartDashboard.putNumber("Turret/Setpoint", this.lazySusanPID.getSetpoint());
    }
}
