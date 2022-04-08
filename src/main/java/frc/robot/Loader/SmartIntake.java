package frc.robot.Loader;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter.FiringPins;
import frc.robot.Util.LEDs.LEDSubsystem;
import frc.robot.Util.LEDs.LEDSubsystem.LEDAnimation;
import frc.robot.Util.LEDs.LEDSubsystem.LEDManager;

public class SmartIntake extends CommandBase {
    protected Intake intake;
    protected FiringPins firingPins;

    protected LEDAnimation noBallsAnim = LEDManager.STRIP0.gradientAnimation(1, 
        Color.kRed,
        Color.kOrangeRed,
        Color.kOrange
    );
    protected LEDAnimation oneBallAnim = LEDManager.STRIP0.gradientAnimation(1, 
        Color.kGreen,
        Color.kYellowGreen,
        Color.kYellow
    );
    protected LEDAnimation twoBallsAnim = LEDManager.STRIP0.gradientAnimation(1, 
        Color.kBlue,
        Color.kBlueViolet,
        Color.kPurple
    );

    // protected ShooterSubsystem shooterSubsystem;
    public SmartIntake(Intake intake, FiringPins firingPins, LEDSubsystem ledSubsystem) {   
        this.intake = intake;
        this.firingPins = firingPins;
        addRequirements(this.intake, ledSubsystem);
        // this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        //intake.enableInnerIntakeMotor();
        this.intake.extendIntakeArmsSolenoid();
        this.intake.enableIntakeArmsMotor();
    }

    @Override
    public void execute() {
        this.intake.enableInnerIntakeMotor();
        if(this.firingPins.hasColor()) { 
            if(this.intake.getIntakeSwitch()) { // TWO balls
                this.intake.disableInnerIntakeMotor();
                this.twoBallsAnim.step();
            } else { // ONE ball
                this.intake.setInnerIntakeMotor(0.5);
                this.oneBallAnim.step();
            }
        } else { // NO balls
            this.noBallsAnim.step();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.disableInnerIntakeMotor();
        this.intake.retractIntakeArmsSolenoid();
        this.intake.disableIntakeArmsMotor();
    }
}

