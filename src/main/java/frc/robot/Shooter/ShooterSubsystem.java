package frc.robot.Shooter;

import java.text.DecimalFormat;
import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.LimeLight;
import frc.robot.Util.sim.LimeLightPoseSim;
import frc.robot.Util.sim.LimeLightSim;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class ShooterSubsystem extends SubsystemBase {
    private final SimableCANSparkMax rightShooterMotor, leftShooterMotor;
    // private final MotorControllerGroup shooterMotors;
    protected NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    protected final RelativeEncoder leftEncoder;
    protected final RelativeEncoder rightEncoder;
    protected final DecimalFormat decFormat = new DecimalFormat("#.#");
    public LimeLight limeLight;
    public LimeLightPoseSim possim;
    public double targetRpm;
    public double currentSpeed = 0;
    private boolean powerDecel = true;

    //PIDs
    protected final PIDController leftShooterPID;
    protected final PIDController rightShooterPID;

    public ShooterSubsystem() {
        rightShooterMotor = new SimableCANSparkMax(Constants.rightShooterMotorID, MotorType.kBrushless);
        leftShooterMotor = new SimableCANSparkMax(Constants.leftShooterMotorID, MotorType.kBrushless);
        // leftShooterMotor.setInverted(true);
        // rightShooterMotor.setInverted(false);

        // shooterMotors = new MotorControllerGroup(rightShooterMotor,
        // leftShooterMotor);
        // shooterMotors.setInverted();
        rightEncoder = rightShooterMotor.getEncoder();
        leftEncoder = leftShooterMotor.getEncoder();

        for (SimableCANSparkMax canSparkMax : List.of(rightShooterMotor, leftShooterMotor)) {
            canSparkMax.restoreFactoryDefaults();
            canSparkMax.setIdleMode(IdleMode.kCoast);
            canSparkMax.setSmartCurrentLimit(45);
        }
        //leftShooterMotor.follow(rightShooterMotor, true);
        // rightShooterMotor.setInverted(true);
        rightShooterMotor.setInverted(true);
        
        leftShooterPID = new PIDController(Constants.diffConstShooter, 0, 0);
        rightShooterPID = new PIDController(Constants.diffConstShooter, 0, 0);
        leftShooterPID.setTolerance(10, 3);
        rightShooterPID.setTolerance(10, 3);
        SmartDashboard.putData("leftShooterPID: ", leftShooterPID);
        SmartDashboard.putData("rightShooterPID: ", rightShooterPID);
    }

    @Override
    public void periodic() {
        leftShooterPID.calculate(leftShooterMotor.getEncoder().getVelocity());
        rightShooterPID.calculate(rightShooterMotor.getEncoder().getVelocity());

        double leftOutput = leftShooterPID.calculate(leftShooterMotor.getEncoder().getVelocity());
        double rightOutput = rightShooterPID.calculate(rightShooterMotor.getEncoder().getVelocity());
        
        leftShooterMotor.set(MathUtil.clamp(leftOutput,powerDecel ? -1: 0,1));
        rightShooterMotor.set(MathUtil.clamp(rightOutput,powerDecel ? -1: 0,1));
        //MathUtil.clamp(output,powerDecel ? -1: 0,1);
    }

    public boolean atTargetRPM() {
        return leftShooterPID.atSetpoint() && rightShooterPID.atSetpoint();
    }
    public void setTargetRPMPID(double tRPM) {
        leftShooterPID.setSetpoint(tRPM);
        rightShooterPID.setSetpoint(tRPM);
    }


    public double getTicksPerMotorRotation() {
        return rightEncoder.getCountsPerRevolution();
    }

    public double getRPM() {
        return rightEncoder.getVelocity();
        // return (rightEncoder.getVelocity() + leftEncoder.getVelocity())/2;
    }

    public void stopMotors() {
        // shooterMotors.stopMotor();
        rightShooterMotor.stopMotor();
    }

    public long getTotalWheelRotations() {
        return (long) rightEncoder.getPosition(); // The conversion factor was previously set
    }

    // public double getSpeed() {
    // // return shooterMotors.get();
    // return leftShooterMotor.get
    // }

    public void setSpeed(double speed) {
        SmartDashboard.putNumber("shooter setpoint speed", speed);
        // shooterMotors.set(speed);
        rightShooterMotor.set(speed);
        // rightShooterMotor.set(speed);
    }

    public void setTargetRPM(double rpm) {
        this.targetRpm = rpm;
    }

    public double getTargetRPM() {
        return this.targetRpm;
    }

    public void setShooterSpeedAndUpdate(double speed) {
        if (speed == 0)
            this.stopMotors();
        else
            this.setSpeed(speed);
        currentSpeed = speed;
    }

    public double getCurrentSpeed() {
        return currentSpeed;
    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return this.simFlywheel.getCurrentDrawAmps() * 2;
        }
        return this.rightShooterMotor.getOutputCurrent() + this.leftShooterMotor.getOutputCurrent();
    }

    /**
     * Simulation Code
     */
    FlywheelSim simFlywheel;
    RevEncoderSimWrapper leftencsim, rightencsim;
    LimeLightSim simLimeLight;
    private boolean simInit = false;

    private void initSim() {
        limeLight = new LimeLight();
        simFlywheel = new FlywheelSim(DCMotor.getNEO(1), Constants.shooterGearRatio, Constants.kSimShooterInertia);

        this.leftencsim = RevEncoderSimWrapper.create(this.leftShooterMotor);
        this.rightencsim = RevEncoderSimWrapper.create(this.rightShooterMotor);
        this.simLimeLight = new LimeLightSim(this.limeLight);
        var hubpos = new Pose2d(7.940, 4.08, new Rotation2d());
        this.possim = new LimeLightPoseSim(simLimeLight, hubpos, Constants.limelightHeight, Constants.hubHeight,
                Constants.azimuthAngle1);
    }

    @Override
    public void simulationPeriodic() {
        if (!simInit) {
            initSim();
            simInit = true;
        }
        simFlywheel.setInputVoltage(rightShooterMotor.get() * RobotController.getInputVoltage());
        simFlywheel.update(Constants.kSimUpdateTime);
        SmartDashboard.putNumber("Right Flywheel Sim", simFlywheel.getAngularVelocityRPM());
        rightencsim.setVelocity(simFlywheel.getAngularVelocityRPM());
        leftencsim.setVelocity(simFlywheel.getAngularVelocityRPM());

        SmartDashboard.putNumber("Right Flywheel motor", getRPM());
    }
}