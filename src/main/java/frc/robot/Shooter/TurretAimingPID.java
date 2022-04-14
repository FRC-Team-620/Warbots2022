package frc.robot.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;

public class TurretAimingPID extends CommandBase {
    protected LazySusanSubsystem lazySusanSubsystem;
    protected Field2d robotFieldWidget;
    protected Supplier<Pose2d> robotbase;
    protected Pose2d prevHubPosition = null;

    public TurretAimingPID(LazySusanSubsystem lazySusanSubsystem, Field2d robotFieldWidget,
            Supplier<Pose2d> robotbase) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        this.robotFieldWidget = robotFieldWidget;
        this.robotbase = robotbase;
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
            if(this.lazySusanSubsystem.getIsGyroLocking()) {
                lazySusanSubsystem.setTurretPositionDegrees(this.robotbase.get().getRotation().plus(lazySusanSubsystem.getRotation()));
            } else {
                lazySusanSubsystem.setTurretPositionDegrees(lazySusanSubsystem.getRotation().minus(Rotation2d.fromDegrees(x)));
            }
            // lazySusanSubsystem.setTurretPositionDegrees(robotbase.get().getRotation().minus(Rotation2d.fromDegrees(x)));
            this.prevHubPosition = this.calculateHubPosition(this.getLocalPose());
        } else if (this.prevHubPosition != null) {
            double dX = calculateHubDeltaX(this.robotbase.get(), this.prevHubPosition);
            lazySusanSubsystem.setTurretPositionDegrees(Rotation2d.fromDegrees(dX));
        }
        // SmartDashboard.putNumber("LimeLight Distance", distance);
        SmartDashboard.putNumber("LimeLight TY", LimeLight.getTY()); // TODO: Remove debug data

    }

    public Pose2d getLocalPose() {
        return new Pose2d(this.robotbase.get().getTranslation(),
                robotbase.get().getRotation().plus(lazySusanSubsystem.getRotation()));
    }

    public Pose2d calculateHubPosition(Pose2d robotPose) {
        double distance = ShooterMath.getDistanceInMeters(Constants.azimuthAngle1, LimeLight.getTY(),
                Constants.limelightHeight, Constants.hubHeight);
        Pose2d hubPosition = robotPose
                .transformBy(new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(-LimeLight.getTX())))
                .transformBy(
                        new Transform2d(new Translation2d(distance, 0), Rotation2d.fromDegrees(0)));
        this.robotFieldWidget.getObject("hub-target").setPose(hubPosition);
        return hubPosition;
    }

    public double calculateHubDeltaX(Pose2d robotPose, Pose2d hubPose) {

        var relativePosition = hubPose.getTranslation().minus(robotPose.getTranslation());
        var angle = Math.toDegrees(Math.atan2(relativePosition.getY(), relativePosition.getX())); // Absolute Field
                                                                                                  // angle
        return angle;
        // return robotPose.minus(hubPosition).getRotation().getDegrees();
    }

    @Override
    public void end(boolean interrupt) {
        LimeLight.setLedMode(LedMode.OFF);
    }
}