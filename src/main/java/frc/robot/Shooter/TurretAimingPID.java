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
    private LazySusanSubsystem lazySusanSubsystem;
    private Field2d robotFieldWidget;
    private Supplier<Pose2d> robotbase;
    private Pose2d prevHubPosition = null;
    private int frames, maxFrames;
    private boolean endOnSuccessfulTracking;

    public TurretAimingPID(LazySusanSubsystem lazySusanSubsystem, Field2d robotFieldWidget,
    Supplier<Pose2d> robotbase, int maxFrames, boolean endOnSuccessfulTracking) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        this.robotFieldWidget = robotFieldWidget;
        this.robotbase = robotbase;
        this.maxFrames = maxFrames;
        this.endOnSuccessfulTracking = endOnSuccessfulTracking;
        addRequirements(lazySusanSubsystem);
    }

    public TurretAimingPID(LazySusanSubsystem lazySusanSubsystem,
    Field2d robotFieldWidget, Supplier<Pose2d> robotbase) {
        this(lazySusanSubsystem, robotFieldWidget, robotbase, -1, false);
    }

    @Override
    public void initialize() {
        this.frames = 0;
        LimeLight.setLedMode(LedMode.ON);
    }

    @Override
    public void execute() {
        this.frames++;
        double x = LimeLight.getTX();
        // SmartDashboard.putNumber("LimeLight Distance", distance);
        SmartDashboard.putNumber("LimeLight TY", LimeLight.getTY()); // TODO: Remove debug data

        // Don't track using the LimeLight if the turret is still turning in the opposite direction
        if (Math.abs(lazySusanSubsystem.getRawSetpoint() - lazySusanSubsystem.getRotationDegrees()) > 180) {
            return;
        }

        if (LimeLight.hasTarget()) {
            if (this.lazySusanSubsystem.getIsGyroLocking()) {
                lazySusanSubsystem.setTurretPosition(this.robotbase.get().getRotation().plus(lazySusanSubsystem.getRotation()).minus(Rotation2d.fromDegrees(x)));
            } else {
                lazySusanSubsystem.setTurretPosition(lazySusanSubsystem.getRotation().minus(Rotation2d.fromDegrees(x)));
            }
            // lazySusanSubsystem.setTurretPositionDegrees(robotbase.get().getRotation().minus(Rotation2d.fromDegrees(x)));
            this.prevHubPosition = this.calculateHubPosition(this.getLocalPose());
        } else if (this.prevHubPosition != null) {
            double deltaX = calculateHubDeltaX(this.robotbase.get(), this.prevHubPosition);

            if (this.lazySusanSubsystem.getIsGyroLocking()) {
                lazySusanSubsystem.setTurretPosition(Rotation2d.fromDegrees(deltaX));
            } else {
                lazySusanSubsystem.setTurretPosition(this.robotbase.get().getRotation().times(-1).plus(Rotation2d.fromDegrees(deltaX)));
            }
        }
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

    @Override
    public boolean isFinished() {
        if (endOnSuccessfulTracking && this.frames < this.maxFrames) {
            return Math.abs(LimeLight.getTX()) < Constants.limeLightTolerance;
        }
        return this.maxFrames == -1 ? false : this.frames >= this.maxFrames;
    }
}
