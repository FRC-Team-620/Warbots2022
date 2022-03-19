package frc.robot.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class ClimberMotorsSubsystem extends SubsystemBase {
    protected final SimableCANSparkMax leftClimberMotor, rightClimberMotor;
    protected final RelativeEncoder encoder;
    protected final SparkMaxLimitSwitch rearLimit;
    protected final DigitalInput climberSensor;


    public ClimberMotorsSubsystem() {

        leftClimberMotor = new SimableCANSparkMax(Constants.leftClimberMotorID, MotorType.kBrushless);
        rightClimberMotor = new SimableCANSparkMax(Constants.rightClimberMotorID, MotorType.kBrushless);
        encoder = rightClimberMotor.getEncoder();
        // rightClimberMotor.limi
        rearLimit = rightClimberMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        leftClimberMotor.restoreFactoryDefaults();
        rightClimberMotor.restoreFactoryDefaults();

        IdleMode mode = IdleMode.kBrake; // brakes
        leftClimberMotor.setIdleMode(mode);
        rightClimberMotor.setIdleMode(mode);

        leftClimberMotor.follow(rightClimberMotor, true);
        climberSensor = new DigitalInput(9);

    }

    public boolean getClimberSensor() {
        return climberSensor.get();
    }
    
    public void setWinchSpeed(double winchSpeed) {
        rightClimberMotor.set(winchSpeed);
    }

    public CANSparkMax getWinchMotor() {
        return rightClimberMotor;
    }
    public RelativeEncoder getWinchEncoder() {
        return encoder;
    }
    public double getWinchPosition() {
        return encoder.getPosition();
    }
    public double getWinchCountsPerRevolution() {
        return encoder.getCountsPerRevolution();
    }

    public boolean hitRearLimitSwitch() {
        return rearLimit.isPressed();
    }

    /**
     * Simulation Code
     */
    private boolean simInit = false;
    private ElevatorSim simClimber;
    private RevEncoderSimWrapper simeEncoder;

    private void initSim() {
        simClimber = new ElevatorSim(DCMotor.getNEO(2), 21, Units.lbsToKilograms(3), Units.inchesToMeters(0.5), 0,Units.feetToMeters(5.5));
        simeEncoder = RevEncoderSimWrapper.create(rightClimberMotor);
    }

    @Override
    public void simulationPeriodic() {
        if (!simInit) {
            initSim();
            simInit = true;
        }
        // simClimber.setInputVoltage(-12);
        // m_elevatorSim.update(0.020);
        var rots = (simClimber.getPositionMeters()/Units.feetToMeters(5.5))* (Constants.winchMaxLimit+1);
        simeEncoder.setDistance(rots); //TODO convert to rotations :/
        SmartDashboard.putNumber("Climber Position", encoder.getPosition());
        SmartDashboard.putNumber("Climber Motor set", rightClimberMotor.get());
        SmartDashboard.putNumber("Climber testpos", simClimber.getPositionMeters());
        // SmartDashboard.putNumber("Turntable Set", lazySusan.get());
        // simeEncoder.setVelocity(simClimber.getVelocityMetersPerSecond());

        // simTurrentRotation = Rotation2d.fromDegrees((encoder.getPosition() / Constants.kSimTurntableGearRatio) * 360);
        // simlazySusan.setInputVoltage(lazySusan.get() * RobotController.getInputVoltage());
        // simlazySusan.update(Constants.kSimUpdateTime);
        // simEncoder.setVelocity(simlazySusan.getAngularVelocityRPM());
        // simEncoder.setDistance(simlazySusan.getAngularPositionRotations() * Constants.kSimTurntableGearRatio / 5);
        // // TODO: Remove magic number 5 that represents the first gear reduction
        
    }
}
