package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class AutoLoad extends CommandBase{
    protected Intake intake;
    protected int frames;

    public AutoLoad(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        frames++;
        if (frames > Constants.frameCountUntilFloat) intake.floatIntakeArms();
    }

    @Override
    public void initialize() {
        this.frames = 0;
        //System.out.println("Loader was turned on");
        //intake.enableInnerIntakeMotor();
        intake.enableIntakeArmsMotor();
        intake.extendIntakeArms();
    }
    
    @Override
    public void end(boolean interrupt) {
        intake.disableInnerIntakeMotor();
        intake.disableIntakeArmsMotor();
        intake.retractIntakeArms();
    }

    @Override
    public boolean isFinished() {
        return this.intake.getIntakeSwitch() || this.frames > 30;
    }
}
