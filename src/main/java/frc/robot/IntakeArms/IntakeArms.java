package frc.robot.IntakeArms;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArms extends SubsystemBase {
    protected Solenoid intakeArmsSolenoid;

    public IntakeArms() {
        intakeArmsSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
        intakeArmsSolenoid.set(false);
    }

    public void extendIntakeArmsSolenoid() {
        intakeArmsSolenoid.set(true);
    }
    
    public void retractIntakeArmsSolenoid() {
        intakeArmsSolenoid.set(false);
    }
}
