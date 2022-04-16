package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class IntakeBall extends CommandBase {
    Intake intake;
    int frameCount = 0;

    public IntakeBall(Intake intake) {   
        this.intake = intake;
    }

    @Override
    public void initialize() {
        frameCount = 0;
        intake.enableInnerIntakeMotor();
        intake.extendIntakeArms();
        intake.enableIntakeArmsMotor();
    }

    @Override
    public void execute() {
        frameCount++;

        if (frameCount > Constants.frameCountUntilFloat) 
            intake.floatIntakeArms();

        // intake.enableInnerIntakeMotor();
        // if(**has ball** (we still need to implement the color sensor)) {
        //     if(**intake pistons NOT resting**) {
        //         intake.disableInnerIntakeMotor();
        //     }
        // }
    }

    @Override
    public void end(boolean interrupted) {
        intake.disableInnerIntakeMotor();
        intake.retractIntakeArms();
        intake.disableIntakeArmsMotor();
    }
}
