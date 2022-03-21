package frc.robot.IntakeArms;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinIntakeArmsMotor extends CommandBase {
    IntakeArmsMotors intakeArmsMotors;
    public SpinIntakeArmsMotor(IntakeArmsMotors intakeArmsMotors) {
        this.intakeArmsMotors = intakeArmsMotors;
    }

    @Override
    public void initialize() {
        intakeArmsMotors.enableIntakeArmsMotor();
    }
}
