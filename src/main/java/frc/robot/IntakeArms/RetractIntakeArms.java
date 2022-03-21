package frc.robot.IntakeArms;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractIntakeArms extends CommandBase {
    IntakeArms intakeArms;
    public RetractIntakeArms(IntakeArms intakeArms) {
        this.intakeArms = intakeArms;
    }

    @Override
    public void initialize() {
        intakeArms.retractIntakeArmsSolenoid();
    }
}
