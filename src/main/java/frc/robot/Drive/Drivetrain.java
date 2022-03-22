package frc.robot.Drive;

import java.util.EnumMap;
import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Util.sim.NavxWrapper;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class Drivetrain extends SubsystemBase {
    public enum Motor {
        RIGHT_BACK, RIGHT_FRONT, LEFT_BACK, LEFT_FRONT;
    }

    private final EnumMap<Motor, SimableCANSparkMax> motors = new EnumMap<>(Motor.class);
    private final EnumMap<Motor, RelativeEncoder> encoders = new EnumMap<>(Motor.class);
    private final MotorControllerGroup rightMotors, leftMotors;
    private final DifferentialDrive diffDrive;
    
    private final AHRS gyro;
    private final DifferentialDriveOdometry odometry;

    public Drivetrain() {
        this.motors.putAll(Map.of(
            Motor.RIGHT_BACK, new SimableCANSparkMax(Constants.rightBackMotorID, MotorType.kBrushless),
            Motor.RIGHT_FRONT, new SimableCANSparkMax(Constants.rightFrontMotorID, MotorType.kBrushless),
            Motor.LEFT_BACK, new SimableCANSparkMax(Constants.leftBackMotorID, MotorType.kBrushless),
            Motor.LEFT_FRONT, new SimableCANSparkMax(Constants.leftFrontMotorID, MotorType.kBrushless)
        ));

        for (Motor motor : Motor.values()) {
            SimableCANSparkMax canSparkMax = this.getMotor(motor);
            canSparkMax.restoreFactoryDefaults();
            canSparkMax.setIdleMode(Constants.idleMode);
            canSparkMax.setOpenLoopRampRate(Constants.rampRate);
            canSparkMax.setSmartCurrentLimit(Constants.currentLimit);
            this.encoders.put(motor, canSparkMax.getEncoder());
        }

        this.rightMotors = new MotorControllerGroup(
            this.getMotor(Motor.RIGHT_FRONT), this.getMotor(Motor.RIGHT_BACK)
        );

        this.leftMotors = new MotorControllerGroup(
            this.getMotor(Motor.LEFT_FRONT), this.getMotor(Motor.LEFT_BACK)
        );

        this.gyro = new AHRS(SerialPort.Port.kMXP);
        this.odometry = new DifferentialDriveOdometry(this.gyro.getRotation2d());

        this.diffDrive = new DifferentialDrive(this.rightMotors, this.leftMotors);
        this.diffDrive.setDeadband(Constants.deadband);
        this.leftMotors.setInverted(true);
    }

    // Motors

    public SimableCANSparkMax getMotor(Motor motor) {
        return this.motors.get(motor);
    }

    public double getSpeed(Motor motor) {
        return this.getMotor(motor).get();
    }

    public void setAllSpeeds(double speed) {
        for(Motor motor : Motor.values()) 
            this.setSpeed(motor, speed);
    }

    public void setSpeed(Motor motor, double speed) {
        this.getMotor(motor).set(speed);
    }

    public void setAllOpenLoopRampRates(double rampRate) {
        for(Motor motor : Motor.values()) 
            this.setOpenLoopRampRate(motor, rampRate);
    }

    public void setOpenLoopRampRate(Motor motor, double rampRate) {
        this.getMotor(motor).setOpenLoopRampRate(rampRate);
    }

    public void setAllIdleModes(IdleMode mode) {
        for(Motor motor : Motor.values()) 
            this.setIdleMode(motor, mode);
    }

    public void setIdleMode(Motor motor, IdleMode mode) {
        this.getMotor(motor).setIdleMode(mode);
    }

    // Encoders

    public RelativeEncoder getEncoder(Motor motor) {
        return this.encoders.get(motor);
    }

    public double getEncoderPos(Motor motor) {
        return this.getEncoder(motor).getPosition();
    }

    public void setAllEncoderPos(double position) {
        for(Motor motor : Motor.values()) 
            this.setEncoderPos(motor, position);
    }

    public void setEncoderPos(Motor motor, double position) {
        this.getEncoder(motor).setPosition(position);
    }

    public double getRPM(Motor motor) {
        return this.getEncoder(motor).getVelocity() * Constants.gearRatio;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            this.getEncoder(Motor.LEFT_FRONT).getVelocity(),
            // TODO: Should this be reading from the back motor?
            this.getEncoder(Motor.RIGHT_BACK).getVelocity()
        );
    }

    // Differential Drive

    public void curvatureDrive(double speed, double rotation, boolean allowTurnInPlace) {
        this.diffDrive.curvatureDrive(speed, rotation, allowTurnInPlace);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        this.getMotor(Motor.LEFT_FRONT).setVoltage(leftVolts);
        this.getMotor(Motor.RIGHT_FRONT).setVoltage(rightVolts);
        diffDrive.feed();
    }

    public void tankDriveSet(double leftSpeed, double rightSpeed) {
        this.getMotor(Motor.LEFT_FRONT).set(leftSpeed);
        this.getMotor(Motor.RIGHT_FRONT).set(rightSpeed);
    }

    public double getDrawnCurrentAmps() {
        if(RobotBase.isSimulation()) {
            return this.m_drivetrainSimulator.getCurrentDrawAmps();
        }
        return this.getMotor(Motor.RIGHT_BACK).getOutputCurrent() +
        this.getMotor(Motor.RIGHT_FRONT).getOutputCurrent() +
        this.getMotor(Motor.LEFT_BACK).getOutputCurrent() +
        this.getMotor(Motor.LEFT_FRONT).getOutputCurrent();
    }

    // Odometry

    public Pose2d getPose() {
        if (RobotBase.isSimulation()) {
            return this.m_drivetrainSimulator.getPose();
        } else {
            return this.odometry.getPoseMeters();
        }
    }

    public double getHeading() {
        return this.gyro.getRotation2d().getDegrees();
    }

    public void resetOdometry(Pose2d pose) {
        this.setAllEncoderPos(0);
        this.odometry.resetPosition(pose, this.gyro.getRotation2d());
    }

    @Override
    public void periodic() {
        this.odometry.update(this.gyro.getRotation2d(), 
        this.getEncoderPos(Motor.LEFT_BACK), 
        this.getEncoderPos(Motor.LEFT_BACK));
    }

    /**
     * Simulation Code
     */

    private NavxWrapper simGryo;
    private DifferentialDrivetrainSim m_drivetrainSimulator;
    private RevEncoderSimWrapper leftencsim;
    private RevEncoderSimWrapper rightencsim;
    private boolean simInit = false;

    private void initSim() {
        LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(Constants.kSimDrivekVLinear,
            Constants.ksimDrivekALinear, Constants.ksimDrivekVAngular,
            Constants.kSimDrivekAAngular);
        m_drivetrainSimulator = new DifferentialDrivetrainSim(
            m_drivetrainSystem, DCMotor.getNEO(2), Constants.gearRatio, Constants.kTrackwidthMeters,
            Units.inchesToMeters(Constants.wheelDiameterInInches / 2), null);

        // Setup Leader Motors
        this.leftencsim = RevEncoderSimWrapper.create(this.getMotor(Motor.LEFT_FRONT));
        this.rightencsim = RevEncoderSimWrapper.create(this.getMotor(Motor.RIGHT_FRONT));

        // Sim Motors
        simGryo = new NavxWrapper();
    }

    @Override
    public void simulationPeriodic() {
        if (!simInit) {
            initSim();
            simInit = true;
        }

        m_drivetrainSimulator.setInputs(
            this.getSpeed(Motor.LEFT_FRONT) * RobotController.getInputVoltage(),
            this.getSpeed(Motor.RIGHT_FRONT) * RobotController.getInputVoltage());
        m_drivetrainSimulator.update(Constants.kSimUpdateTime);
        this.leftencsim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
        this.leftencsim.setVelocity(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

        this.rightencsim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
        this.rightencsim.setVelocity(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

        simGryo.getYawGyro().setAngle(-m_drivetrainSimulator.getHeading().getDegrees()); // TODO add Gyo Vel support
    }
}
