package frc.robot.IntakeArms;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReverseSpinIntakeArmsMotor extends CommandBase{
    IntakeArmsMotors intakeArmsMotors;
    public ReverseSpinIntakeArmsMotor(IntakeArmsMotors intakeArmsMotors) {
        this.intakeArmsMotors = intakeArmsMotors;
    }

    @Override
    public void initialize() {
        intakeArmsMotors.reverseIntakeArmsMotor();
    }
}
