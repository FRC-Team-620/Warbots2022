package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Controls.ControlBoard;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;

public class LimelightSpinUp extends CommandBase {
    protected ShooterSubsystem shooterSubsystem;

    public LimelightSpinUp(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        LimeLight.setLedMode(LedMode.ON);
    }

    @Override
    public void execute() {
        double y = LimeLight.getTY();
        // double distance = ShooterMath.getDistanceInMeters(Constants.azimuthAngle1, y, Constants.limelightHeight, Constants.hubHeight);
        // double targetRPM = ShooterMath.metersToRPM(distance);
        
        // TODO: Use the below RPM value once the table is working
        double targetRPM = Constants.rpmMap.getInterpolated(y);

        this.shooterSubsystem.setTargetRPM(targetRPM);
        ControlBoard.setOperatorRumble(this.getWithinTolerance());
    }

    private boolean getWithinTolerance(){
        return ShooterMath.withinTolerance(
            this.shooterSubsystem.getRPM(), 
            this.shooterSubsystem.getTargetRPM(), 
            Constants.shooterVibrationTolerance);
    }

    @Override
    public void end(boolean interrupt) {
        this.shooterSubsystem.stopMotors();
        LimeLight.setLedMode(LedMode.OFF);
        ControlBoard.setOperatorRumble(false);
    }
}