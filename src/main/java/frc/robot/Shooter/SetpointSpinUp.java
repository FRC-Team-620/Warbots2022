package frc.robot.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;

public class SetpointSpinUp extends CommandBase {
    protected ShooterSubsystem shooterSubsystem;
    protected double offsetY;
    private Timer timer;

    public SetpointSpinUp(ShooterSubsystem shooterSubsystem, double offsetY) {

        this.shooterSubsystem = shooterSubsystem;
        this.offsetY = offsetY;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setTargetRPM(Constants.rpmMap.getInterpolated(offsetY));
    }
    
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupt) {
        shooterSubsystem.setTargetRPM(0);
        LimeLight.setLedMode(LedMode.OFF);
    }
}
