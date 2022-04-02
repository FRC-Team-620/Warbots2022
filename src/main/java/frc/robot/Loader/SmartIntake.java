package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.ShooterSubsystem;

public class SmartIntake extends CommandBase {
    protected Intake intake;
    protected FiringPins firingPins;
    protected ShooterSubsystem shooterSubsystem;
    public SmartIntake(Intake intake, FiringPins firingPins, ShooterSubsystem shooterSubsystem) {   
        this.intake = intake;
        this.firingPins = firingPins;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        // intake.enableInnerIntakeMotor();
        intake.extendIntakeArmsSolenoid();
        // intake.enableIntakeArmsMotor();
    }

    @Override
    public void execute() {
        
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
        intake.retractIntakeArmsSolenoid();
        intake.disableIntakeArmsMotor();
    }
}

