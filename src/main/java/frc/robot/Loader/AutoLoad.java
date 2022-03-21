package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.InnerIntake.InnerIntake;
import frc.robot.IntakeArms.IntakeArms;
import frc.robot.IntakeArms.IntakeArmsMotor;

public class AutoLoad extends CommandBase{
    protected InnerIntake innerIntake;
    protected IntakeArms intakeArms;
    protected IntakeArmsMotor intakeArmsMotor;
    protected int frames;

    public AutoLoad(InnerIntake innerIntake, IntakeArms intakeArms, IntakeArmsMotor intakeArmsMotor) {
        this.innerIntake = innerIntake;
        this.intakeArms = intakeArms;
        this.intakeArmsMotor = intakeArmsMotor;
        addRequirements(innerIntake, intakeArms, intakeArmsMotor);
    }

    @Override
    public void execute() {
        frames++;
    }

    @Override
    public void initialize() {
        this.frames = 0;
        //System.out.println("Loader was turned on");
        innerIntake.enableInnerIntakeMotor();
        intakeArmsMotor.enableIntakeArmsMotor();
        intakeArms.extendIntakeArmsSolenoid();
    }
    
    @Override
    public void end(boolean interrupt) {
        innerIntake.disableInnerIntakeMotor();
        intakeArmsMotor.disableIntakeArmsMotor();
        intakeArms.retractIntakeArmsSolenoid();
    }

    @Override
    public boolean isFinished() {
        return frames > 750;
    }
}
