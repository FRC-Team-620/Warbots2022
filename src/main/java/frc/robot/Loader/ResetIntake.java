package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.InnerIntake.DeactivateInnerIntake;
import frc.robot.InnerIntake.InnerIntake;
import frc.robot.IntakeArms.DeactivateIntakeArmsMotor;
import frc.robot.IntakeArms.IntakeArms;
import frc.robot.IntakeArms.IntakeArmsMotor;
import frc.robot.IntakeArms.RetractIntakeArms;

public class ResetIntake extends ParallelCommandGroup {
    private InnerIntake innerIntake;
    private IntakeArms intakeArms;
    private IntakeArmsMotor intakeArmsMotor;

    public ResetIntake(InnerIntake innerIntake, IntakeArms intakeArms, IntakeArmsMotor intakeArmsMotor) {
        this.innerIntake = innerIntake;
        this.intakeArms = intakeArms;
        this.intakeArmsMotor = intakeArmsMotor;
        addCommands(
            new DeactivateInnerIntake(innerIntake),
            new RetractIntakeArms(intakeArms),
            new DeactivateIntakeArmsMotor(intakeArmsMotor)
        );
    }
}
