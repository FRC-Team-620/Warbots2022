package frc.robot.Util.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Loader.Intake;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Util.LEDs.LEDSubsystem.LEDAnimation;
import frc.robot.Util.LEDs.LEDSubsystem.LEDManager;

public class LEDIdleCommand extends CommandBase {
    protected Intake intake;
    protected FiringPins firingPins;
    protected LazySusanSubsystem lazySusanSubsystem;

    protected LEDAnimation noBallsAnimCalibrated = LEDManager.STRIP0.fadeAnimation(1,
        60, 
        Color.kYellow,
        Color.kWhite
    );
    protected LEDAnimation noBallsAnimUncalibrated = LEDManager.STRIP0.fadeAnimation(1,
        60, 
        Color.kForestGreen, 
        Color.kWhite
    );
    protected LEDAnimation oneBallAnim = LEDManager.STRIP0.gradientAnimation(1, 
        Color.kRed,
        Color.kOrangeRed,
        Color.kOrange
    );
    // protected LEDAnimation twoBallsAnim = LEDManager.STRIP0.gradientAnimation(1, 
    //     Color.kBlue,
    //     Color.kBlueViolet,
    //     Color.kPurple
    // );

    // protected LEDAnimation noBallsAnim = LEDManager.STRIP0.gradientAnimation(1, 
    //     Color.kRed,
    //     Color.kOrangeRed,
    //     Color.kOrange
    // );
    // protected LEDAnimation twoBallsAnim = LEDManager.STRIP0.gradientAnimation(1, 
    //     Color.kBlue,
    //     Color.kBlueViolet,
    //     Color.kPurple
    // );
    // protected LEDAnimation oneBallAnim = LEDAnimation.transposeBlinking(0.04, 
    //     twoBallsAnim, 
    //     noBallsAnim
    // );

    public LEDIdleCommand(LEDSubsystem ledSubsystem, Intake intake, FiringPins firingPins, LazySusanSubsystem lazySusanSubsystem) {
        this.intake = intake;
        this.firingPins = firingPins;
        this.lazySusanSubsystem = lazySusanSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        // if(this.firingPins.hasColor()) { 
        if(this.intake.getIntakeSwitch()) { // TWO balls
            this.oneBallAnim.step();
            // } else { // ONE ball
            //     this.oneBallAnim.step();
            // }
        } else if(this.lazySusanSubsystem.getIsCal()) { // NO balls
            this.noBallsAnimCalibrated.step();
        } else {
            this.noBallsAnimUncalibrated.step();
        }
    }
}