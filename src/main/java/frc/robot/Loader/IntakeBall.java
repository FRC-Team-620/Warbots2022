package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class IntakeBall extends CommandBase {
    Intake intake;
    public IntakeBall(Intake intake) {   
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.enableInnerIntakeMotor();
        intake.extendIntakeArmsSolenoid();
        intake.enableIntakeArmsMotor();
    }

    @Override
    public void end(boolean interrupted) {
        intake.disableInnerIntakeMotor();
        intake.retractIntakeArmsSolenoid();
        intake.disableIntakeArmsMotor();
    }


}
