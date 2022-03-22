package frc.robot.Shooter;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;

public class SetpointSpinUp extends CommandBase {
    protected ShooterSubsystem shooterSubsystem;
    protected XboxController operatorXbox;
    protected double setPoint;

    public SetpointSpinUp(ShooterSubsystem shooterSubsystem, XboxController operatorXbox, 
        double setPoint) {

        this.shooterSubsystem = shooterSubsystem;
        this.operatorXbox = operatorXbox;
        this.setPoint = setPoint;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        // double acceleration = shooterSubsystem.calculateAccelerationPController(setPoint);
        // shooterSubsystem.setSpeed(shooterSubsystem.getCurrentSpeed() + acceleration);
        shooterSubsystem.setTargetRPM(setPoint);
        double rumble = 0;
        // if (shooterSubsystem.getRPM() > (1-Constants.shooterVibrationTolerance)*shooterSubsystem.getTargetRPM()
        //     && shooterSubsystem.getRPM() < (1+Constants.shooterVibrationTolerance)*shooterSubsystem.getTargetRPM()) {
        //     rumble = Constants.operatorRumble;
        // }
        if(ShooterMath.withinTolerance(shooterSubsystem.getRPM(), 
            shooterSubsystem.getTargetRPM(), Constants.shooterVibrationTolerance)) {
            
            rumble = Constants.operatorRumble;
        }
        operatorXbox.setRumble(RumbleType.kLeftRumble, rumble);
        operatorXbox.setRumble(RumbleType.kRightRumble, rumble);
    }

    @Override
    public void end(boolean interrupt) {
        shooterSubsystem.stopMotors();
        LimeLight.setLedMode(LedMode.OFF);
        operatorXbox.setRumble(RumbleType.kLeftRumble, 0);
        operatorXbox.setRumble(RumbleType.kRightRumble, 0);
    }
}
