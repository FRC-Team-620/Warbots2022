package frc.robot.InnerIntake;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReverseSpinInnerIntake extends CommandBase {
    InnerIntake innerIntake;
    public ReverseSpinInnerIntake(InnerIntake innerIntake) {
        addRequirements(innerIntake);
        this.innerIntake = innerIntake;
    }

    @Override
    public void initialize() {
        innerIntake.reverseInnerIntakeMotor();
    }
}
