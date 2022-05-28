package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Shooter.FiringPins;

public class SmartIntake extends CommandBase {
    private Intake intake;
    private FiringPins firingPins;
    private int frames = 0;

    // protected ShooterSubsystem shooterSubsystem;
    public SmartIntake(Intake intake, FiringPins firingPins) {   
        this.intake = intake;
        this.firingPins = firingPins;
        addRequirements(this.intake);
        // this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        frames = 0;
        // intake.enableInnerIntakeMotor();
        // this.intake.extendIntakeArms();
        // this.intake.enableIntakeArmsMotor();
        this.intake.enableInnerIntakeMotor();
    }

    // @Override
    // public void execute() {
    //     if (++this.frames > Constants.frameCountUntilFloat) 
    //         intake.floatIntakeArms();
        
    //     // if(this.firingPins.hasColor()) { 
    //     //     if(this.intake.getIntakeSwitch()) { // TWO balls
    //     //         this.intake.disableInnerIntakeMotor();
    //     //     } else { // ONE ball
    //     //         this.intake.setInnerIntakeMotor(0.9);
    //     //     }
    //     if (this.intake.getIntakeSwitch()) {
    //         this.intake.disableInnerIntakeMotor();
    //     } else {} // NO balls (add code here if needed)
    // }

    @Override
    public void end(boolean interrupted) {
        this.intake.disableInnerIntakeMotor();
        // this.intake.retractIntakeArms();
        // this.intake.disableIntakeArmsMotor();
    }
}
