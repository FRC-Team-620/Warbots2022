package frc.robot.Util.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.LEDs.LEDSubsystem.LEDAnimation;

public class LEDIdleCommand extends CommandBase {
    protected int frames = 0;
    protected LEDAnimation idleAnimation = 
        LEDSubsystem.LEDManager.STRIP0.colorBlocksAnimation(0.05, new int[]{2, 1, 2}, 
            Color.kRed,
            Color.kWhite
        );

    public LEDIdleCommand(LEDSubsystem ledSubsystem) {
        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        this.idleAnimation.step();
    }
}