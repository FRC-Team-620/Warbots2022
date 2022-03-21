package frc.robot.IntakeArms;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinIntakeArmsMotor extends CommandBase {
    IntakeArmsMotor intakeArmsMotors;
    public SpinIntakeArmsMotor(IntakeArmsMotor intakeArmsMotors) {
        addRequirements(intakeArmsMotors);
        this.intakeArmsMotors = intakeArmsMotors;
    }

    @Override
    public void initialize() {
        intakeArmsMotors.enableIntakeArmsMotor();
    }
}
