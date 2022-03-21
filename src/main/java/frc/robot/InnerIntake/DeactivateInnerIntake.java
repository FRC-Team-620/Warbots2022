package frc.robot.InnerIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DeactivateInnerIntake extends CommandBase {
    InnerIntake innerIntake;
    public DeactivateInnerIntake(InnerIntake innerIntake) {
        this.innerIntake = innerIntake;
    }

    @Override
    public void initialize() {
        innerIntake.disableInnerIntakeMotor();
    }
}
