package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class OuttakeBall extends CommandBase {
    Intake intake;
    public OuttakeBall(Intake intake) {   
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.reverseInnerIntakeMotor();
        intake.retractIntakeArmsSolenoid();
        intake.disableIntakeArmsMotor();
    }

    @Override
    public void end(boolean interrupted) {
        intake.disableInnerIntakeMotor();
        intake.retractIntakeArmsSolenoid();
        intake.disableIntakeArmsMotor();
    }


}
