package frc.robot.Shooter;

import java.text.DecimalFormat;
import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Controls.ControlBoard;
import frc.robot.Util.LimeLight;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax rightShooterMotor, leftShooterMotor;
    // private final MotorControllerGroup shooterMotors;
    protected final RelativeEncoder leftEncoder;
    protected final RelativeEncoder rightEncoder;
    protected final DecimalFormat decFormat = new DecimalFormat("#.#");
    public double targetRpm;
    private boolean isBackward;
    private boolean powerDecel = true;
    private double offsetSpeed = 0;

    //PIDs
    protected final PIDController leftShooterPID;
    protected final PIDController rightShooterPID;
    // Feedforward
    private final double kS = -0.07488, kV = 0.12385, kA = 0.020886;
    protected final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA); //TODO: SysID characterize 
    // TODO: Tune PID loops more for lower RPMs
	private final double kP = 0.000001, kI = 0.003500, kD = 0.010000;//0.00025 0.0004

    // When setting any non-speed value, make sure to set the isBackward boolean!

    public ShooterSubsystem() {
        rightShooterMotor = new CANSparkMax(Constants.rightShooterMotorID, MotorType.kBrushless);
        leftShooterMotor = new CANSparkMax(Constants.leftShooterMotorID, MotorType.kBrushless);
        // leftShooterMotor.setInverted(true);
        // rightShooterMotor.setInverted(false);

        // shooterMotors = new MotorControllerGroup(rightShooterMotor,
        // leftShooterMotor);
        // shooterMotors.setInverted();
        rightEncoder = rightShooterMotor.getEncoder();
        leftEncoder = leftShooterMotor.getEncoder();

        for (CANSparkMax canSparkMax : List.of(rightShooterMotor, leftShooterMotor)) {
            canSparkMax.restoreFactoryDefaults();
            canSparkMax.setIdleMode(IdleMode.kCoast);
            canSparkMax.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        }
        //leftShooterMotor.follow(rightShooterMotor, true);
        // rightShooterMotor.setInverted(true);
        leftShooterMotor.setInverted(true);

        isBackward = false;
        
        leftShooterPID = new PIDController(kP, kI, kD);
        rightShooterPID = new PIDController(kP, kI, kD);
        leftShooterPID.setTolerance(10, 10);
        rightShooterPID.setTolerance(10, 10);
        // SmartDashboard.putData(leftShooterPID);
        SmartDashboard.putNumber("Shooter/lowPoweredShotRPM", Constants.lowPoweredShotRPM);
    }

    @Override
    public void periodic() {

        boolean hasTargetAndInRange = LimeLight.hasTarget() && Constants.rpmMap.isKeyInBounds(LimeLight.getTY());

        ControlBoard.setDriverHighFreqRumble(hasTargetAndInRange);
        
        // Debug Force both Pid loops to same setpoint; //TODO: Prob need to remove
        //rightShooterPID.setSetpoint(leftShooterPID.getSetpoint());

        leftShooterPID.calculate(leftShooterMotor.getEncoder().getVelocity());
        rightShooterPID.calculate(rightShooterMotor.getEncoder().getVelocity());

        double leftOutputVoltage = leftShooterPID.calculate(leftShooterMotor.getEncoder().getVelocity()) + 
            feedForward.calculate(leftShooterPID.getSetpoint()/60);
        double rightOutputVoltage = rightShooterPID.calculate(rightShooterMotor.getEncoder().getVelocity()) + 
            feedForward.calculate(rightShooterPID.getSetpoint()/60);
        if (!isBackward) {
            leftShooterMotor.setVoltage(MathUtil.clamp(leftOutputVoltage, powerDecel || leftShooterPID.getSetpoint() <= 0 ? 0 : -13, 13));
            rightShooterMotor.setVoltage(MathUtil.clamp(rightOutputVoltage, powerDecel || rightShooterPID.getSetpoint() <= 0 ? 0 : -13, 13));
        } else {
            leftShooterMotor.setVoltage(-13);
            rightShooterMotor.setVoltage(-13);
        }

        ControlBoard.setDriverLowFreqRumble(hasTargetAndInRange && this.getWithinTolerance());
        //MathUtil.clamp(output,powerDecel ? -1: 0,1);
    }

    private boolean getWithinTolerance(){
        return ShooterMath.withinTolerance(
            this.getRPM(), 
            this.getTargetRPM(), 
            Constants.shooterVibrationTolerance);
    }

    public double getSpeed() {
        return rightShooterMotor.get();
    }

    public void setSpeed(double speed) {
        this.isBackward = speed < 0;
        rightShooterMotor.set(speed);
        leftShooterMotor.set(speed);
    }

    public double getOffsetSpeed() {
        return offsetSpeed;
    }

    public void setOffsetSpeed(double offsetSpeed) {
        this.offsetSpeed = offsetSpeed;
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

    public double getLeftRPM() {
        return leftEncoder.getVelocity();
    }
    public double getRightRPM() {
        return rightEncoder.getVelocity();
    }

    public double getSetpoint() {
        return leftShooterPID.getSetpoint();
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
        this.isBackward = false;
        leftShooterPID.setSetpoint(tRPM);
        rightShooterPID.setSetpoint(tRPM);
    }

    public double getTargetRPM() {
        return rightShooterPID.getSetpoint();
    }

    public void setIsBackwards(boolean iS) {
        isBackward = iS;
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
        return this.rightShooterMotor.getOutputCurrent() + this.leftShooterMotor.getOutputCurrent();
    }
}