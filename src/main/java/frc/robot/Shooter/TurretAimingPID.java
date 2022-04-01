package frc.robot.Shooter;

import java.util.ResourceBundle.Control;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Controls.ControlBoard;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;

public class TurretAimingPID extends CommandBase {
    protected LazySusanSubsystem lazySusanSubsystem;
    protected Field2d robotFieldWidget;
    protected Supplier<Pose2d> robotbase;
    protected Pose2d prevHubPosition = null;

    public TurretAimingPID(LazySusanSubsystem lazySusanSubsystem, Field2d robotFieldWidget, Supplier<Pose2d> robotbase) {
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
            boolean inRange = LimeLight.inRange();
            lazySusanSubsystem.setTurretPositionDegrees(lazySusanSubsystem.getRotation().minus(Rotation2d.fromDegrees(x)));
            ControlBoard.setOperatorRumble(!inRange);
            ControlBoard.setDriverRumble(!inRange);

            this.prevHubPosition = this.calculateHubPosition(this.getLocalPose());
        } else {
            ControlBoard.setOperatorRumble(true);
            ControlBoard.setDriverRumble(true);

            if(this.prevHubPosition != null) {
                Pose2d robotPosition = this.getLocalPose();
                double dX = calculateHubDeltaX(robotPosition, this.prevHubPosition);
                lazySusanSubsystem.setTurretPositionDegrees(lazySusanSubsystem.getRotation().minus(Rotation2d.fromDegrees(dX)));
            }
        }
        // SmartDashboard.putNumber("LimeLight Distance", distance);
        SmartDashboard.putNumber("LimeLight TY", LimeLight.getTY()); //TODO: Remove debug data

        // SmartDashboard.putNumber("Test rotation", calculateHubDeltaX(robotPosition, hubPosition));

        // asdf.robotFieldWidget.getObject("hub-target").setPose(testpos);;
    }

    public Pose2d getLocalPose() {
        return new Pose2d(this.robotbase.get().getTranslation(),  robotbase.get().getRotation().plus(lazySusanSubsystem.getRotation()));
    }

    public Pose2d calculateHubPosition(Pose2d robotPose) {
        double distance = ShooterMath.getDistanceInMeters(Constants.azimuthAngle1, LimeLight.getTY(), Constants.limelightHeight, Constants.hubHeight);
        Pose2d hubPosition = robotPose.transformBy(new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(-LimeLight.getTX()))).transformBy(
            new Transform2d(new Translation2d(distance, 0), Rotation2d.fromDegrees(0)));
        this.robotFieldWidget.getObject("hub-target").setPose(hubPosition);
        return hubPosition;
    }

    public double calculateHubDeltaX(Pose2d robotPose, Pose2d hubPosition) {
        return robotPose.minus(hubPosition).getRotation().getDegrees();
    }

    @Override
    public void end(boolean interrupt) {
        LimeLight.setLedMode(LedMode.OFF);
        ControlBoard.setOperatorRumble(false);
        ControlBoard.setDriverRumble(false);
    }
}
