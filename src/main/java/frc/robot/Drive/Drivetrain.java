package frc.robot.Drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.NavxWrapper;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class Drivetrain extends SubsystemBase {

    private SimableCANSparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    private DifferentialDrive drive;
    private AHRS navx;
    private RelativeEncoder leftFrontEncoder, rightFrontEncoder;
    private DifferentialDriveOdometry odometry;

    // PUBLIC
    public Drivetrain() {
        initMotors();
        initDrive();
        initSensors();
    }

    public void curvatureInput(double speed, double rotation, boolean quickTurn) {
        drive.curvatureDrive(speed, -rotation, quickTurn);
    }

    public void setEncoderPos(double pos) {
        frontLeftMotor.getEncoder().setPosition(pos);
        frontRightMotor.getEncoder().setPosition(pos);
        rearLeftMotor.getEncoder().setPosition(pos);
        rearRightMotor.getEncoder().setPosition(pos);

    }

    public double getHeading() { // TODO: Remove Use Odom class
        return navx.getAngle();
    }

    public void tankDriveSet(double leftSpeed, double rightSpeed) {
        frontLeftMotor.set(leftSpeed);
        frontRightMotor.set(rightSpeed);
        // drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDriveSet(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    // average the two encoders for this later
    public double getLeftSpeed() { // TODO: Remove Use Odom class
        return leftFrontEncoder.getVelocity();
    }

    public double getLeftPosition() {// TODO: Remove Use Odom class
        return leftFrontEncoder.getPosition();
    }

    public double getRightSpeed() {//TODO: Remove Use Odom class
    return rightFrontEncoder.getVelocity();
    }

    public double getRightPosition() {//TODO: Remove Use Odom class
    return rightFrontEncoder.getPosition();
    }

    public void setBrake(boolean brake) {
        IdleMode mode;
        if (brake) {
            mode = IdleMode.kBrake;
        } else {
            mode = IdleMode.kCoast;
        }

        frontRightMotor.setIdleMode(mode);
        frontLeftMotor.setIdleMode(mode);
        rearRightMotor.setIdleMode(mode);
        rearLeftMotor.setIdleMode(mode);

    }

    @Override
    public void periodic() {
        odometry.update(navx.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());
        SmartDashboard.putNumber("Heading", navx.getYaw());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // PRIVATE
    private void initMotors() {
        frontLeftMotor = new SimableCANSparkMax(DriveConstants.frontLeftMotorID, MotorType.kBrushless);
        frontRightMotor = new SimableCANSparkMax(DriveConstants.frontRightMotorID, MotorType.kBrushless);
        rearLeftMotor = new SimableCANSparkMax(DriveConstants.rearLeftMotorID, MotorType.kBrushless);
        rearRightMotor = new SimableCANSparkMax(DriveConstants.rearRightMotorID, MotorType.kBrushless);

        rearRightMotor.follow(frontRightMotor);
        rearLeftMotor.follow(frontLeftMotor);

        frontRightMotor.setInverted(false);
        frontLeftMotor.setInverted(true);

        int currentLimit = 60;
        frontLeftMotor.setSmartCurrentLimit(currentLimit);
        frontRightMotor.setSmartCurrentLimit(currentLimit);
        rearLeftMotor.setSmartCurrentLimit(currentLimit);
        rearRightMotor.setSmartCurrentLimit(currentLimit);

        frontRightMotor.setOpenLoopRampRate(DriveConstants.driveRampRate);
        frontLeftMotor.setOpenLoopRampRate(DriveConstants.driveRampRate);

        setBrake(true);
    }

    private void initDrive() {
        drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
        drive.setMaxOutput(DriveConstants.driveTrainMaxSpeed);
    }

    private void initSensors() {
        navx = new AHRS(Port.kMXP);

        leftFrontEncoder = frontLeftMotor.getEncoder();
        rightFrontEncoder = frontRightMotor.getEncoder();
        odometry = new DifferentialDriveOdometry(navx.getRotation2d());
    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return this.m_drivetrainSimulator.getCurrentDrawAmps();
        }
        return this.frontLeftMotor.getOutputCurrent() + this.frontRightMotor.getOutputCurrent()
                + this.rearLeftMotor.getOutputCurrent() + this.rearRightMotor.getOutputCurrent();
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
        LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
                Constants.kSimDrivekVLinear,
                Constants.ksimDrivekALinear, Constants.ksimDrivekVAngular,
                Constants.kSimDrivekAAngular);
        m_drivetrainSimulator = new DifferentialDrivetrainSim(
                m_drivetrainSystem, DCMotor.getNEO(2), Constants.gearRatio, Constants.kTrackwidthMeters,
                Units.inchesToMeters(Constants.wheelDiameterInInches / 2), null);

        // Setup Leader Motors
        this.leftencsim = RevEncoderSimWrapper.create(this.frontLeftMotor);
        this.rightencsim = RevEncoderSimWrapper.create(this.frontRightMotor);

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
                this.frontLeftMotor.get() * RobotController.getInputVoltage(),
                this.frontRightMotor.get() * RobotController.getInputVoltage());
        m_drivetrainSimulator.update(Constants.kSimUpdateTime);
        this.leftencsim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
        this.leftencsim.setVelocity(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

        this.rightencsim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
        this.rightencsim.setVelocity(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

        simGryo.getYawGyro().setAngle(-m_drivetrainSimulator.getHeading().getDegrees()); // TODO add Gyo Vel support
    }

}
