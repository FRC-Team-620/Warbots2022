package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.ShooterSubsystem;

public class SmartInnerIntake extends CommandBase {
    protected Intake intake;
    protected FiringPins firingPins;
    protected ShooterSubsystem shooterSubsystem;
    protected boolean pastValue;
    protected boolean pastPastValue;
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
        boolean currentValue = intake.getIntakeSwitch();
        if (currentValue == false && pastValue && pastPastValue) {
            return true;
        } else {
            pastPastValue = pastValue;
            pastValue = currentValue;
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.disableInnerIntakeMotor();
        intake.retractIntakeArmsSolenoid();
        intake.disableIntakeArmsMotor();
    }
}

