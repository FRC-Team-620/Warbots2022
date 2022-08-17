package frc.robot.Util.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.LEDs.LEDSubsystem.LEDAnimation;
import frc.robot.Util.LEDs.LEDSubsystem.LEDManager;

public class LEDIdleCommand extends CommandBase {

    protected LEDAnimation rainbow = LEDManager.STRIP0.gradientAnimation(
        1,
        Color.kRed,
        Color.kOrange,
        Color.kYellow,
        Color.kGreen,
        Color.kCyan,
        Color.kBlue,
        Color.kPurple
    );

    protected LEDAnimation madison = LEDManager.STRIP0.fadeTwoAnimation(
        1,
        60,
        Color.kBlack,
        Color.kDarkRed
    );

    public LEDIdleCommand(LEDSubsystem ledSubsystem) {
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        this.madison.step();
    }
}
