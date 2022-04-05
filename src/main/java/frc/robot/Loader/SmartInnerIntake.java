package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.ShooterSubsystem;

public class SmartInnerIntake extends CommandBase {
    protected Intake intake;
    protected FiringPins firingPins;
    protected ShooterSubsystem shooterSubsystem;
    public SmartInnerIntake(Intake intake) {   
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.enableInnerIntakeMotor();
        // intake.extendIntakeArmsSolenoid();
        // intake.enableIntakeArmsMotor();
    }

    @Override
    public void execute() {
       
    }

    @Override
    public boolean isFinished() {
        return intake.getIntakeSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        intake.disableInnerIntakeMotor();
        intake.retractIntakeArmsSolenoid();
        intake.disableIntakeArmsMotor();
    }
}

