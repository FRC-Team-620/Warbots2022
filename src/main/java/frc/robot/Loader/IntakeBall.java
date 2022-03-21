package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.InnerIntake.InnerIntake;
import frc.robot.InnerIntake.SpinInnerIntake;
import frc.robot.IntakeArms.ExtendIntakeArms;
import frc.robot.IntakeArms.IntakeArms;
import frc.robot.IntakeArms.IntakeArmsMotor;
import frc.robot.IntakeArms.SpinIntakeArmsMotor;

public class IntakeBall extends ParallelCommandGroup {
    public IntakeBall(InnerIntake innerIntake, IntakeArms intakeArms, IntakeArmsMotor intakeArmsMotor) {   
        addCommands(
            new SpinInnerIntake(innerIntake),
            new ExtendIntakeArms(intakeArms),
            new SpinIntakeArmsMotor(intakeArmsMotor)
        );
    }
}
