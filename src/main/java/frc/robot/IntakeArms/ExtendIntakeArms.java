package frc.robot.IntakeArms;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendIntakeArms extends CommandBase {
    IntakeArms intakeArms;
    public ExtendIntakeArms(IntakeArms intakeArms) {
        this.intakeArms = intakeArms;
    }

    @Override
    public void initialize() {
        intakeArms.extendIntakeArmsSolenoid();
    }
}
