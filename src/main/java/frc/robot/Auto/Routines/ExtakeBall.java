package frc.robot.Auto.Routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Loader.Intake;

public class ExtakeBall extends SequentialCommandGroup {
    private final double extakeSpeed = 1;

    public ExtakeBall(Intake intake) {
        addCommands(
            new InstantCommand(() -> intake.setInnerIntakeMotor(extakeSpeed)),
            new WaitCommand(2),
            new InstantCommand(intake::disableInnerIntakeMotor)
        );
    }
}
