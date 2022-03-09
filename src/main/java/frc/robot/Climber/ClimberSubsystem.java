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
    // protected final CANSparkMax leftClimberMotor, rightClimberMotor;
    // protected final RelativeEncoder encoder;
    protected final Solenoid hangingSolenoid;
    protected final Solenoid armsSolenoid;

    //Sim
    private ElevatorSim climbeSim;
    RevEncoderSimWrapper leftencsim,rightencsim;
    public ClimberSubsystem() {

        hangingSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
        armsSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    }

    public Solenoid getHangingSolenoid() {
        return hangingSolenoid;
    }
    public Solenoid getArmsSolenoid() {
        return armsSolenoid;
    }

    public void setHangingSolenoid(boolean b) {
        this.getHangingSolenoid().set(b);
    }
    public void setArmsSolenoid(boolean b) {
        this.getArmsSolenoid().set(b);
    }

}
