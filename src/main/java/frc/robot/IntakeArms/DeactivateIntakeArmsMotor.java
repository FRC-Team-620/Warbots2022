package frc.robot.IntakeArms;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DeactivateIntakeArmsMotor extends CommandBase {
    IntakeArmsMotors intakeArmsMotors;
    public DeactivateIntakeArmsMotor(IntakeArmsMotors intakeArmsMotors) {
        this.intakeArmsMotors = intakeArmsMotors;
    }

    @Override
    public void initialize() {
        intakeArmsMotors.disableIntakeArmsMotor();
    }
}
