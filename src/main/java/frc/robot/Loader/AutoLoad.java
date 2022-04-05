package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;

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
    }

    @Override
    public void initialize() {
        this.frames = 0;
        //System.out.println("Loader was turned on");
        //intake.enableInnerIntakeMotor();
        intake.enableIntakeArmsMotor();
        intake.extendIntakeArmsSolenoid();
    }
    
    @Override
    public void end(boolean interrupt) {
        intake.disableInnerIntakeMotor();
        intake.disableIntakeArmsMotor();
        intake.retractIntakeArmsSolenoid();
    }

    @Override
    public boolean isFinished() {
        return frames > 750;
    }
}
