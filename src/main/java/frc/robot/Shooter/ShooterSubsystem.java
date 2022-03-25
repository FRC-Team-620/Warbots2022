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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.LimeLightPoseSim;
import frc.robot.Util.sim.LimeLightSim;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class ShooterSubsystem extends SubsystemBase {
    private final SimableCANSparkMax rightShooterMotor, leftShooterMotor;
    // private final MotorControllerGroup shooterMotors;
    protected final RelativeEncoder leftEncoder;
    protected final RelativeEncoder rightEncoder;
    protected final DecimalFormat decFormat = new DecimalFormat("#.#");
    public double targetRpm;
    public double currentSpeed = 0;
    private boolean powerDecel = true;

    //PIDs
    protected final PIDController leftShooterPID;
    protected final PIDController rightShooterPID;
    // TODO: Tune PID loops more for lower RPMs
	private final double kP = 0.00025, kI = 0.0004;
    private double testRPM;

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
        leftShooterMotor.setInverted(true);
        
        leftShooterPID = new PIDController(kP, kI, 0);
        rightShooterPID = new PIDController(kP, kI, 0);
        leftShooterPID.setTolerance(10, 10);
        rightShooterPID.setTolerance(10, 10);
        SmartDashboard.putData(leftShooterPID);
        SmartDashboard.putData(rightShooterPID);
        SmartDashboard.putNumber("Test RPM: ", testRPM);
    }

    @Override
    public void periodic() {
        leftShooterPID.calculate(leftShooterMotor.getEncoder().getVelocity());
        rightShooterPID.calculate(rightShooterMotor.getEncoder().getVelocity());

        double leftOutput = leftShooterPID.calculate(leftShooterMotor.getEncoder().getVelocity());
        double rightOutput = rightShooterPID.calculate(rightShooterMotor.getEncoder().getVelocity());
        
        leftShooterMotor.set(MathUtil.clamp(leftOutput, powerDecel || leftShooterPID.getSetpoint() <= 0 ? 0 : -1, 1));
        rightShooterMotor.set(MathUtil.clamp(rightOutput, powerDecel || rightShooterPID.getSetpoint() <= 0 ? 0 : -1, 1));

        // int x = 0;
        // if (leftShooterPID.atSetpoint()) {
        //     x = 5000;
        // }

        SmartDashboard.putNumber("Flywheel Right RPM", rightShooterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Flywheel Left RPM", leftShooterMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Flywheel Left atTarget", x);
        SmartDashboard.putNumber("Flywheel Right Setpoint", rightShooterPID.getSetpoint());
        SmartDashboard.putNumber("Flywheel Left Setpoint", leftShooterPID.getSetpoint());
        //MathUtil.clamp(output,powerDecel ? -1: 0,1);
    }

    public double getTestRPM() {
        return SmartDashboard.getNumber("Test RPM: ", 0);
    }

    public double getSpeed() {
        return rightShooterMotor.get();
    }

    public void setSpeed(double speed) {
        rightShooterMotor.set(speed);
        leftShooterMotor.set(speed);
    }
    
    public boolean atTargetRPM() {
        return leftShooterPID.atSetpoint() && rightShooterPID.atSetpoint();
    }
   
    public double getTicksPerMotorRotation() {
        return rightEncoder.getCountsPerRevolution();
    }

    public double getRPM() {
        // return rightEncoder.getVelocity();
        return (rightEncoder.getVelocity() + leftEncoder.getVelocity())/2;
    }

    public void stopMotors() {
		setTargetRPM(0);
        // shooterMotors.stopMotor();
        // rightShooterMotor.stopMotor();
    }

    public long getTotalWheelRotations() {
        return (long) rightEncoder.getPosition(); // The conversion factor was previously set
    }

    // public double getSpeed() {
    // // return shooterMotors.get();
    // return leftShooterMotor.get
    // }

    //public void setSpeed(double speed) {
        //SmartDashboard.putNumber("shooter setpoint speed", speed);
        // shooterMotors.set(speed);
        //rightShooterMotor.set(speed);
        //leftShooterMotor.set(speed);
        // rightShooterMotor.set(speed);
    //}

    public void setTargetRPM(double tRPM) {
        leftShooterPID.setSetpoint(tRPM);
        rightShooterPID.setSetpoint(tRPM);
    }

    public double getTargetRPM() {
        return rightShooterPID.getSetpoint();
    }

    //public void setShooterSpeedAndUpdate(double speed) {
        //if (speed == 0)
            //this.stopMotors();
        //else
            //this.setSpeed(speed);
        //currentSpeed = speed;
    //}

    //public double getCurrentSpeed() {
        //return currentSpeed;
    //}

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return this.simFlywheelLeft.getCurrentDrawAmps() + this.simFlywheelRight.getCurrentDrawAmps();
        }
        return this.rightShooterMotor.getOutputCurrent() + this.leftShooterMotor.getOutputCurrent();
    }

    /**
     * Simulation Code
     */
    FlywheelSim simFlywheelLeft,simFlywheelRight;
    RevEncoderSimWrapper leftencsim, rightencsim;
    public LimeLightSim simLimeLight;
    public LimeLightPoseSim possim;
    private boolean simInit = false;

    private void initSim() {
        simFlywheelLeft = new FlywheelSim(DCMotor.getNEO(1), Constants.shooterGearRatio, Constants.kSimShooterInertia);
        simFlywheelRight = new FlywheelSim(DCMotor.getNEO(1), Constants.shooterGearRatio, Constants.kSimShooterInertia);
        this.leftencsim = RevEncoderSimWrapper.create(this.leftShooterMotor);
        this.rightencsim = RevEncoderSimWrapper.create(this.rightShooterMotor);
        this.simLimeLight = new LimeLightSim();
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
        // SmartDashboard.putNumber("Right Flywheel motor set", rightShooterMotor.get());
        simFlywheelRight.setInputVoltage(rightShooterMotor.get() * RobotController.getInputVoltage());
        simFlywheelRight.update(Constants.kSimUpdateTime);

        simFlywheelLeft.setInputVoltage(leftShooterMotor.get() * RobotController.getInputVoltage());
        simFlywheelLeft.update(Constants.kSimUpdateTime);
        // SmartDashboard.putNumber("Right Flywheel Sim", rightencsim.getAngularVelocityRPM());
        rightencsim.setVelocity(simFlywheelRight.getAngularVelocityRPM());
        leftencsim.setVelocity(simFlywheelLeft.getAngularVelocityRPM());

        // SmartDashboard.putNumber("Right Flywheel motor", getRPM());
    }
}