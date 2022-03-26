package frc.robot.Shooter;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Controls.ControlBoard;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;

public class SetpointSpinUp extends CommandBase {
    protected ShooterSubsystem shooterSubsystem;
    protected double setPoint;
    private Timer timer;

    public SetpointSpinUp(ShooterSubsystem shooterSubsystem, double setPoint) {

        this.shooterSubsystem = shooterSubsystem;
        this.setPoint = setPoint;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setTargetRPM(setPoint);
        ControlBoard.setOperatorRumble(getWithinTolerance());
    }

    private boolean getWithinTolerance() {
        return ShooterMath.withinTolerance(
            this.shooterSubsystem.getRPM(), 
            this.shooterSubsystem.getTargetRPM(), 
            Constants.shooterVibrationTolerance);
    }
    
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupt) {
        shooterSubsystem.setTargetRPM(0);
        LimeLight.setLedMode(LedMode.OFF);
        ControlBoard.setOperatorRumble(false);
    }
}