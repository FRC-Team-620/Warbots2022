package frc.robot.InnerIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinInnerIntake extends CommandBase {
    InnerIntake innerIntake;
    public SpinInnerIntake(InnerIntake innerIntake) {
        addRequirements(innerIntake);
        this.innerIntake = innerIntake;
    }

    @Override
    public void initialize() {
        innerIntake.enableInnerIntakeMotor();
    }
}
