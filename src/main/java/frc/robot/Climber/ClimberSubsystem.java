package frc.robot.Climber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    // protected final CANSparkMax leftClimberMotor, rightClimberMotor;
    // protected final RelativeEncoder encoder;
    protected final Solenoid hangingSolenoid;
    protected final Solenoid armsSolenoid;
    

    public ClimberSubsystem() {

        hangingSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
        armsSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    }

    @Override
    public void periodic() {
        // System.out.println(getClimberSensor());
    }


    public Solenoid getHangingSolenoid() {
        return hangingSolenoid;
    }
    public Solenoid getArmsSolenoid() {
        return armsSolenoid;
    }
    // public Solenoid getBumperSolenoid() {
    //     return bumperSolenoid;
    // }
    


    public void setHangingSolenoid(boolean b) {
        this.getHangingSolenoid().set(b);
    }
    public void setArmsSolenoid(boolean b) {
        this.getArmsSolenoid().set(b);
    }
    // public void setBumperSolenoid(boolean b) {
    //     this.getBumperSolenoid().set(b);
    // }

}
