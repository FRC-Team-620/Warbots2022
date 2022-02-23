// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import com.kauailabs.navx.AHRSProtocol.MagCalData;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.NavxWrapper;
import frc.robot.Util.sim.RevEncoderSimWrapper;
import frc.robot.Util.sim.SimSparkMax;

public class Drivetrain extends SubsystemBase {
  protected final SimableCANSparkMax rightBackMotor, leftBackMotor, rightFrontMotor, leftFrontMotor;
  protected final DifferentialDrive diffDrive;

  double kTrackWidth = 0.762;
  double kWheelRadius = 0.0508;
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
      0.3);
  private final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(
      m_drivetrainSystem, DCMotor.getNEO(2), 8, kTrackWidth, kWheelRadius, null);
  // private final EncoderSim m_leftEncoderSim;
  // private final EncoderSim m_rightEncoderSim;
  RevEncoderSimWrapper leftencsim;
  RevEncoderSimWrapper rightencsim;
  protected final RelativeEncoder leftBackEncoder, leftFrontEncoder, rightBackEncoder, rightFrontEncoder;
  protected final double countsPerMotorRevolution;
  protected double openLoopRampRate = 0.2;
  // The gyro sensor
  protected final AHRS gyro;
  private NavxWrapper simGryo;

  // Odometry class for tracking robot pose
  protected final DifferentialDriveOdometry odometry;

  // Network Table
  protected final NetworkTableInstance instance;
  protected final NetworkTable table;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    rightBackMotor = new SimableCANSparkMax(Constants.rightBackMotorID, MotorType.kBrushless);
    leftBackMotor = new SimableCANSparkMax(Constants.leftBackMotorID, MotorType.kBrushless);
    rightFrontMotor = new SimableCANSparkMax(Constants.rightFrontMotorID, MotorType.kBrushless);
    leftFrontMotor = new SimableCANSparkMax(Constants.leftFrontMotorID, MotorType.kBrushless);
    // m_leftEncoderSim = new EncoderSim(leftFrontMotor.getEncoder());
    leftencsim = RevEncoderSimWrapper.create(this.leftFrontMotor);
    rightencsim = RevEncoderSimWrapper.create(this.rightFrontMotor);
    // SimableCANSparkMax t;
    rightBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    leftFrontMotor.restoreFactoryDefaults();

    IdleMode mode = IdleMode.kBrake; // brakes
    rightBackMotor.setIdleMode(mode);
    rightFrontMotor.setIdleMode(mode);
    leftBackMotor.setIdleMode(mode);
    leftFrontMotor.setIdleMode(mode);

    double openLoopRampRate = 0.2; // 0.6 sec to full velocity
    rightBackMotor.setOpenLoopRampRate(openLoopRampRate);
    rightFrontMotor.setOpenLoopRampRate(openLoopRampRate);
    leftBackMotor.setOpenLoopRampRate(openLoopRampRate);
    leftFrontMotor.setOpenLoopRampRate(openLoopRampRate);

    int currentLimit = 45; // maxmium amps
    rightBackMotor.setSmartCurrentLimit(currentLimit);
    rightFrontMotor.setSmartCurrentLimit(currentLimit);
    leftBackMotor.setSmartCurrentLimit(currentLimit);
    leftFrontMotor.setSmartCurrentLimit(currentLimit);

    rightBackMotor.follow(rightFrontMotor, false); // false means not inverted, and true means inverted
    leftBackMotor.follow(leftFrontMotor, false);

    // Encoder creation
    leftBackEncoder = rightFrontMotor.getEncoder();
    leftFrontEncoder = rightBackMotor.getEncoder();
    rightBackEncoder = leftFrontMotor.getEncoder();
    rightFrontEncoder = leftBackMotor.getEncoder();

    countsPerMotorRevolution = leftBackEncoder.getCountsPerRevolution();
    // this choice of encoder is arbitrary -- any other encoder would work just as
    // well

    var conversionFactor = Constants.gearRatio * Constants.wheelDiameterInInches * Constants.inchesToMetersFactor
        * Math.PI;
    leftBackEncoder.setPositionConversionFactor(conversionFactor);
    leftFrontEncoder.setPositionConversionFactor(conversionFactor);
    rightFrontEncoder.setPositionConversionFactor(conversionFactor);
    rightBackEncoder.setPositionConversionFactor(conversionFactor);

    gyro = new AHRS(SerialPort.Port.kMXP);
    simGryo = new NavxWrapper();

    diffDrive = new DifferentialDrive(rightFrontMotor, leftFrontMotor);
    diffDrive.setDeadband(0.05); // minmal signal
    leftFrontMotor.setInverted(true);
    resetEncoders();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    // NetworkTable instantiation
    instance = NetworkTableInstance.getDefault();
    table = instance.getTable("/SmartDashboard");

    // ShuffleboardTab tab = Shuffleboard.getTab("Smart Dashboard");
  }

  public double getOpenLoopRampRate() {
    return openLoopRampRate;
  }

  public void setOpenLoopRampRate(double oLRR) {
    openLoopRampRate = oLRR; // 0.6 sec to full velocity
    rightBackMotor.setOpenLoopRampRate(openLoopRampRate);
    rightFrontMotor.setOpenLoopRampRate(openLoopRampRate);
    leftBackMotor.setOpenLoopRampRate(openLoopRampRate);
    leftFrontMotor.setOpenLoopRampRate(openLoopRampRate);
  }

  public void setMotorMode(IdleMode mode) {
    rightBackMotor.setIdleMode(mode);
    rightFrontMotor.setIdleMode(mode);
    leftBackMotor.setIdleMode(mode);
    leftFrontMotor.setIdleMode(mode);
  }

  public Pose2d getPose() {
    if(RobotBase.isSimulation()){
      return m_drivetrainSimulator.getPose();
    }else{
      return odometry.getPoseMeters();
    }
   
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getDistance(leftBackEncoder), getDistance(rightBackEncoder));
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double rightVolts, double leftVolts) {
    rightFrontMotor.setVoltage(leftVolts);
    leftFrontMotor.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public CANSparkMax getMotor(int idx) {
    idx = (idx - 1) % 4 + 1;
    switch (idx) {
      case 1:
        return rightBackMotor;
      case 2:
        return rightFrontMotor;
      case 3:
        return leftBackMotor;
    }
    return leftFrontMotor;
  }

  public RelativeEncoder getEncoder(int idx) {
    idx = (idx - 1) % 4 + 1;
    switch (idx) {
      case 1:
        return leftFrontEncoder;
      case 2:
        return leftBackEncoder;
      case 3:
        return rightFrontEncoder;
    }
    return rightBackEncoder;
  }

  public void curvatureInput(double speed, double rotation, boolean isCurvatureDrive) {
    diffDrive.curvatureDrive(speed, rotation, isCurvatureDrive);
  }

  public void rightFrontMotorDrive(double x) {
    rightFrontMotor.set(x);
  }

  public void leftFrontMotorDrive(double x) {
    leftFrontMotor.set(x);
  }

  public void rightBackMotorDrive(double x) {
    rightBackMotor.set(x);
  }

  public void leftBackMotorDrive(double x) {
    leftBackMotor.set(x);
  }

  public double getRPM(int idx) {
    idx = (idx - 1) % 4 + 1;
    return getEncoder(idx).getVelocity() * Constants.gearRatio;
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getDistance(leftBackEncoder), getDistance(rightBackEncoder));
  }

  protected double getDistance(RelativeEncoder enc) {
    return enc.getPosition() * enc.getPositionConversionFactor();
  }

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putNumber("Motor out 1", this.leftFrontMotor.get());
    SmartDashboard.putNumber("Motor out 2", this.rightFrontMotor.get());
    // this.rightFrontMotor.();
    ;
    m_drivetrainSimulator.setInputs(
        this.leftFrontMotor.get(), // TODO Voltage Compensation
        this.rightFrontMotor.get());
    m_drivetrainSimulator.update(0.02);
    var leftencsim = RevEncoderSimWrapper.create(this.leftFrontMotor);
    var rightencsim = RevEncoderSimWrapper.create(this.rightFrontMotor);
    leftencsim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    leftencsim.setVelocity(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

    rightencsim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    rightencsim.setVelocity(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

    simGryo.getYawGyro().setAngle(-m_drivetrainSimulator.getHeading().getDegrees()); //TODO add Gyo Vel support
    // SmartDashboard.putNumber("Navx yaw", gyro.getYaw());
    // SmartDashboard.putNumber("Navx yaw V", gyro.getVelocityY());
    //TODO Current Simulation
    // gyro.
    // m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    // m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    // m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    // m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    super.simulationPeriodic();
  }
}
