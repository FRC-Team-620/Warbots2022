package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OuttakeBall extends CommandBase {
    Intake intake;
    public OuttakeBall(Intake intake) {   
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.reverseInnerIntakeMotor();
        intake.retractIntakeArms();
        intake.disableIntakeArmsMotor();
    }

    @Override
    public void end(boolean interrupted) {
        intake.disableInnerIntakeMotor();
        intake.retractIntakeArms();
        intake.disableIntakeArmsMotor();
    }


}
