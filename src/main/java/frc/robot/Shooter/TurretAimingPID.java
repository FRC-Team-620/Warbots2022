package frc.robot.Shooter;

import java.util.ResourceBundle.Control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Controls.ControlBoard;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;

public class TurretAimingPID extends CommandBase {
    protected LazySusanSubsystem lazySusanSubsystem;

    public TurretAimingPID(LazySusanSubsystem lazySusanSubsystem) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        addRequirements(lazySusanSubsystem);
    }

    @Override
    public void initialize() {
        LimeLight.setLedMode(LedMode.ON);
    }

    @Override
    public void execute() {
        double x = LimeLight.getTX();
        if (LimeLight.hasTarget()) {
            lazySusanSubsystem.setTurretPositionDegrees(lazySusanSubsystem.getRotation().minus(Rotation2d.fromDegrees(x)));
        }
        SmartDashboard.putNumber("LimeLight Distance", ShooterMath.getDistanceInMeters(Constants.azimuthAngle1, LimeLight.getTY(), Constants.limelightHeight, Constants.hubHeight));
        SmartDashboard.putNumber("LimeLight TY", LimeLight.getTY()); //TODO: Remove debug data
    }

    @Override
    public void end(boolean interrupt) {
        LimeLight.setLedMode(LedMode.OFF);
        ControlBoard.setOperatorRumble(false);
        ControlBoard.setDriverRumble(false);
    }
}
