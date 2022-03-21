package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.InnerIntake.InnerIntake;
import frc.robot.InnerIntake.ReverseSpinInnerIntake;
import frc.robot.IntakeArms.DeactivateIntakeArmsMotor;
import frc.robot.IntakeArms.IntakeArms;
import frc.robot.IntakeArms.IntakeArmsMotor;
import frc.robot.IntakeArms.RetractIntakeArms;

public class OuttakeBall extends ParallelCommandGroup {
    public OuttakeBall(InnerIntake innerIntake, IntakeArms intakeArms, IntakeArmsMotor intakeArmsMotor) {
        addCommands(
            new ReverseSpinInnerIntake(innerIntake),
            new RetractIntakeArms(intakeArms),
            new DeactivateIntakeArmsMotor(intakeArmsMotor)
        );
    }
}
