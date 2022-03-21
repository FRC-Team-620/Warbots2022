package frc.robot.IntakeArms;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DeactivateIntakeArmsMotor extends CommandBase {
    IntakeArmsMotor intakeArmsMotors;
    public DeactivateIntakeArmsMotor(IntakeArmsMotor intakeArmsMotors) {
        addRequirements(intakeArmsMotors);
        this.intakeArmsMotors = intakeArmsMotors;
    }

    @Override
    public void initialize() {
        intakeArmsMotors.disableIntakeArmsMotor();
    }
}
