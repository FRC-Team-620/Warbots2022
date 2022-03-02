package frc.robot.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class ClimberSubsystem extends SubsystemBase {
    protected final SimableCANSparkMax leftClimberMotor, rightClimberMotor;
    protected final RelativeEncoder encoder;
    protected final Solenoid hangingSolenoid;
    protected final Solenoid armsSolenoid;

    //Sim
    private ElevatorSim climbeSim;
    RevEncoderSimWrapper leftencsim,rightencsim;
    public ClimberSubsystem() {

        leftClimberMotor = new SimableCANSparkMax(Constants.leftClimberMotorID, MotorType.kBrushless);
        rightClimberMotor = new SimableCANSparkMax(Constants.rightClimberMotorID, MotorType.kBrushless);
        encoder = rightClimberMotor.getEncoder();

        leftClimberMotor.restoreFactoryDefaults();
        rightClimberMotor.restoreFactoryDefaults();

        IdleMode mode = IdleMode.kBrake; // brakes
        leftClimberMotor.setIdleMode(mode);
        rightClimberMotor.setIdleMode(mode);

        leftClimberMotor.follow(rightClimberMotor, false);

        hangingSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
        armsSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

        // leftShooterMotor.setInverted(false);
        // rightShooterMotor.setInverted(false);
        if(RobotBase.isSimulation()){
            initSim();
        }
    }
    public void initSim(){
        climbeSim = new ElevatorSim(DCMotor.getNEO(2), Constants.kSimClimberGearRatio, Constants.kSimRobotWeight, Constants.kSimClimberDrumSize, 0, Constants.ksimClimberMaxHeight);
        leftencsim = RevEncoderSimWrapper.create(leftClimberMotor);
        rightencsim = RevEncoderSimWrapper.create(rightClimberMotor);
    }

    public void setWinchSpeed(double winchSpeed) {
        rightClimberMotor.set(winchSpeed);
    }

    public CANSparkMax getWinchMotor() {
        return rightClimberMotor;
    }

    public Solenoid getHangingSolenoid() {
        return hangingSolenoid;
    }
    public Solenoid getArmsSolenoid() {
        return armsSolenoid;
    }
    @Override
    public void simulationPeriodic() {
       climbeSim.setInputVoltage(rightClimberMotor.get() * RobotController.getInputVoltage());
       climbeSim.update(Constants.kSimUpdateTime);
       rightencsim.setVelocity(climbeSim.getVelocityMetersPerSecond());
       rightencsim.setDistance(climbeSim.getPositionMeters());

       leftencsim.setVelocity(climbeSim.getVelocityMetersPerSecond());
       rightencsim.setDistance(climbeSim.getPositionMeters());

       //TODO: Convert to correct pulses/conversions
       //TODO: handle encoder zeroing?
    }


}
