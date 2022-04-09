package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Shooter.FiringPins;

public class SmartIntake extends CommandBase {
    protected Intake intake;
    protected FiringPins firingPins;
    int frameCount = 0;

    // protected ShooterSubsystem shooterSubsystem;
    public SmartIntake(Intake intake, FiringPins firingPins) {   
        this.intake = intake;
        this.firingPins = firingPins;
        addRequirements(this.intake);
        // this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        frameCount = 0;
        //intake.enableInnerIntakeMotor();
        this.intake.extendIntakeArms();
        this.intake.enableIntakeArmsMotor();
    }

    @Override
    public void execute() {
        frameCount++;
        if (frameCount > Constants.frameCountUntilFloat) intake.floatIntakeArms();

        this.intake.enableInnerIntakeMotor();
        if(this.firingPins.hasColor()) { 
            if(this.intake.getIntakeSwitch()) { // TWO balls
                this.intake.disableInnerIntakeMotor();
            } else { // ONE ball
                this.intake.setInnerIntakeMotor(0.5);
            }
        } else {} // NO balls (add code here if needed)
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.disableInnerIntakeMotor();
        this.intake.retractIntakeArms();
        this.intake.disableIntakeArmsMotor();
    }
}

