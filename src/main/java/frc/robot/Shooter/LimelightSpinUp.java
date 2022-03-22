package frc.robot.Shooter;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;

public class LimelightSpinUp extends CommandBase {
    protected ShooterSubsystem shooterSubsystem;
    protected XboxController operatorXbox;

    public LimelightSpinUp(ShooterSubsystem shooterSubsystem, XboxController operatorXbox) {
        this.shooterSubsystem = shooterSubsystem;
        this.operatorXbox = operatorXbox;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        LimeLight.setLedMode(LedMode.ON);
    }

    @Override
    public void execute() {
        double y = LimeLight.getTY();
        double distance = ShooterMath.getDistanceInMeters(Constants.azimuthAngle1, y, Constants.limelightHeight, Constants.hubHeight);
        double targetRPM = ShooterMath.metersToRPM(distance);
    
        shooterSubsystem.setTargetRPM(targetRPM);
        
        double rumble = 0;
        if (shooterSubsystem.getRPM() > (1-Constants.shooterVibrationTolerance)*targetRPM
            && shooterSubsystem.getRPM() < (1+Constants.shooterVibrationTolerance)*targetRPM) {
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
