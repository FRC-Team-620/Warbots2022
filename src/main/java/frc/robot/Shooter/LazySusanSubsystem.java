package frc.robot.Shooter;

import java.util.function.Supplier;

import javax.swing.UIDefaults.LazyInputMap;

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
    // private Rotation2d turretRotation;
    private double desiredRotation;
    private final Supplier<Pose2d> robotBasePose;
    private boolean isGyroLocking;
    private final double countsToDegreesFactor = (1.0 / (5.23 * 5.23)) * (20.0 / 156.0) * 360.0;
    private double maxModSpeed = 0.8;
    private double modSpeed = 1;
    private final double kP = 0.038000, kI = 0.2, kD = 0.000400;// KI0.00004 kP = 0.060000, kI = 0.003000, TODO: Tune PID
                                                              // Loop
    private boolean isCal;
    private double offsetAngle = -4;
    // private boolean isDisabled;
    public DigitalInput calSwitch;
    private final double lowLimitDegrees = -195;
    private final double highLimitDegrees = 220;
    private final double errorMargin = 2;
    private double maxIntegrator = 0.7;

    private boolean isHubTracking;
    // Left 45.690002, Right -45.356651, AbsoluteMaxRange 90
    // private double turntableThresh = 35;

    public LazySusanSubsystem(Supplier<Pose2d> robotBasePose) {
        this.isGyroLocking = false;
        this.isHubTracking = false;
        this.robotBasePose = robotBasePose;
        this.motor = new SimableCANSparkMax(Constants.lazySusanID, MotorType.kBrushless);
        this.calSwitch = new DigitalInput(Constants.calSwitchID);
        this.motor.restoreFactoryDefaults();
        this.encoder = this.motor.getEncoder();
        encoder.setPositionConversionFactor(countsToDegreesFactor); //* 0.914
        this.encoder.setPosition(Constants.stowedDegrees);
        IdleMode mode = IdleMode.kBrake; // brakes
        this.motor.setIdleMode(mode);
        this.motor.setSmartCurrentLimit(18);

        // lazySusan.setInverted(true);
        lazySusanPID = new PIDController(kP, kI, kD);
        lazySusanPID.setTolerance(1);
        lazySusanPID.setIntegratorRange(-this.maxIntegrator, this.maxIntegrator);
        // turretRotation =encoder.getPosition();
        // turretRotation = desiredRotation = Constants.stowedDegrees;
        isCal = false;
        // isDisabled =
        SmartDashboard.putData("Turret/PIDController[12]", lazySusanPID);
        SmartDashboard.putNumber("Turret/offsetAngle", offsetAngle);
    }

    @Override
    public void periodic() {

        //desiredRotation = SmartDashboard.getNumber("Turret/Desired Rotation", desiredRotation);
        //offsetAngle = SmartDashboard.getNumber("Turret/offsetAngle", offsetAngle);
        //this.maxIntegrator = SmartDashboard.getNumber("Turret/maxIntegrator", this.maxIntegrator);
        //lazySusanPID.setIntegratorRange(-this.maxIntegrator, this.maxIntegrator);

        double degrees = isGyroLocking
                ? Rotation2d.fromDegrees(desiredRotation).minus(robotBasePose.get().getRotation()).getDegrees()
                : desiredRotation;

        degrees = findClosestSolution(degrees, getRotationDegrees()); //Solve Two Solution Problem
        lazySusanPID.setSetpoint(MathUtil.clamp(degrees, lowLimitDegrees, highLimitDegrees));
        double pidOutput = MathUtil.clamp(lazySusanPID.calculate(encoder.getPosition()), -1, 1);

        if (encoder.getPosition() > highLimitDegrees) {
            pidOutput = MathUtil.clamp(pidOutput, -1, 0);
        }

        if (encoder.getPosition() < lowLimitDegrees) {
            pidOutput = MathUtil.clamp(pidOutput, 0, 1);
        }

        // motor.set(pidOutput * modSpeed);

        motor.set(
                MathUtil.clamp(pidOutput, -this.modSpeed * this.maxModSpeed, this.modSpeed * this.maxModSpeed));
        // System.out.println("FNIEGOIN: " + Math.abs(degrees -
        // this.turretRotation.getDegrees()));

        SmartDashboard.putNumber("Turret/Raw Encoder", encoder.getPosition());
        SmartDashboard.putNumber("Turret/Motor Percentage", motor.get());
        SmartDashboard.putBoolean("Turret/Is Calibrated", isCal);
        SmartDashboard.putBoolean("Turret/Calibration Switch", islimitSwitchPressed());
        SmartDashboard.putNumber("Turret/Desired Rotation", desiredRotation);
        SmartDashboard.putNumber("Turret/Setpoint", this.lazySusanPID.getSetpoint());
        SmartDashboard.putBoolean("Turret/Is Gyro Locking", isGyroLocking);
        SmartDashboard.putBoolean("Turret/Is Hub Tracking", isHubTracking);
        SmartDashboard.putNumber("Turret/ModSpeed", modSpeed); 
        SmartDashboard.putNumber("Turret/maxIntegrator", this.maxIntegrator);     
        
    }

    public double findClosestSolution(double targetRotation, double currentRotation){
        double solution2 = targetRotation - Math.signum(targetRotation) * 360;
        double distance1 = Math.abs(desiredRotation - this.getRotationDegrees());
        double distance2 = Math.abs(solution2 - this.getRotationDegrees());
        return distance2 < distance1 && solution2 > lowLimitDegrees && solution2 < highLimitDegrees
        ? solution2 : targetRotation; // Find which of the two solutions is closest
    }

    public double getOffsetAngle() {
        return offsetAngle;
    }
    
    public void setOffsetAngle(double oA) {
        offsetAngle = oA;
    }


    public void setSmartCurrentLimit(int currentLimit) {
        this.motor.setSmartCurrentLimit(currentLimit);
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public void setMotorMode(IdleMode m) {
        this.motor.setIdleMode(m);
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
        if (isGyroLocking) {
            setTurretPosition(robotBasePose.get().getRotation().plus(getDesiredRotation()));
        } else {
            setTurretPosition(getDesiredRotation().minus(robotBasePose.get().getRotation()));
        }
        this.isGyroLocking = isGyroLocking;
    }

    public boolean getIsHubTracking() {
        return this.isHubTracking;
    }

    public void setIsHubTracking(boolean track) {
        this.isHubTracking = track;
    }

    public boolean getIsCal() {
        return isCal;
    }

    public void setIsCal(boolean isCal) {
        this.isCal = isCal;
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(encoder.getPosition());
    }

    public double getRotationDegrees() {
        return encoder.getPosition();
    }

    public Rotation2d getDesiredRotation() {
        return Rotation2d.fromDegrees(desiredRotation);
    }

    public double getDesiredRotationDegrees() {
        return desiredRotation;
    }

    public void setTurretPosition(Rotation2d rot) {
        desiredRotation = rot.getDegrees();
        // this.setTurretPosition(degrees/countToDegreesFactor);
    }

    public void setTurretPositionDegrees(double rot) {
        desiredRotation = MathUtil.clamp(rot, lowLimitDegrees, highLimitDegrees);
    }

    public double getRawSetpoint() {
        return lazySusanPID.getSetpoint();
    }

    public boolean atTurretPosition() {
        return lazySusanPID.atSetpoint();
    }

    public void setEncoderPosition(double p) {
        encoder.setPosition(p);
    }

    public void setMotorSpeed(double speed) {
        this.motor.set(speed);
    }

    public void stop() {
        lazySusanPID.reset();
        lazySusanPID.setSetpoint(encoder.getPosition());
        this.setTurretPositionDegrees(this.encoder.getPosition());
    }

    public void setHomePosition() {
        setIsGyroLocking(false);
        double limitPos = 186.5;// -45 205.5
        setEncoderPosition(limitPos);
        lazySusanPID.reset();
        lazySusanPID.setSetpoint(limitPos);
        System.out.println("lazySusanPID: " + lazySusanPID.getSetpoint());
        this.setTurretPositionDegrees(limitPos);
        // setTurretPositionDegrees(Rotation2d.fromDegrees(limitPos));
        // turretRotation = Rotation2d.fromDegrees(limitPos);
        // stop();
        setIsCal(true);
    }

    public boolean islimitSwitchPressed() {
        return calSwitch.get();
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
    private boolean simInit = false;
    private double simlastpos = 0;
    private double simoffset = 0;

    private void initSim() {
        simlazySusan = new DCMotorSim(DCMotor.getNeo550(1), (1.0) / (((1.0 / 25.0) * (20.0 / 156.0)* 0.914)),
                Constants.kSimTurntableInertia); // TODO: add gear ratio
        simEncoder = RevEncoderSimWrapper.create(this.motor);
    }

    @Override
    public void simulationPeriodic() {
        if (!simInit) {
            initSim();
            simInit = true;
        }
        simlazySusan.setInputVoltage(motor.get() * RobotController.getInputVoltage());
        simlazySusan.update(Constants.kSimUpdateTime);
        simEncoder.setVelocity(simlazySusan.getAngularVelocityRPM());
        if (simEncoder.getPosition() != simlastpos) {
            System.out.println(simEncoder.getPosition() + " " + simlastpos);
            simoffset = (simlazySusan.getAngularPositionRotations() * 360) - simEncoder.getPosition();
            System.out.println("Encoder Position changed" + simoffset);
        }

        simEncoder.setDistance((simlazySusan.getAngularPositionRotations() * 360) - simoffset);
        // TODO: Remove magic number 5 that represents the first gear reduction
        simlastpos = simEncoder.getPosition();
        SmartDashboard.putNumber("Turret/Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Turret/Sim rotation", simlazySusan.getAngularPositionRotations() * 360);
        SmartDashboard.putNumber("Turret/Setpoint", this.lazySusanPID.getSetpoint());
    }
}
