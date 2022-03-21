package frc.robot.IntakeArms;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReverseSpinIntakeArmsMotor extends CommandBase{
    IntakeArmsMotor intakeArmsMotors;
    public ReverseSpinIntakeArmsMotor(IntakeArmsMotor intakeArmsMotors) {
        addRequirements(intakeArmsMotors);
        this.intakeArmsMotors = intakeArmsMotors;
    }

    @Override
    public void initialize() {
        intakeArmsMotors.reverseIntakeArmsMotor();
    }
}
