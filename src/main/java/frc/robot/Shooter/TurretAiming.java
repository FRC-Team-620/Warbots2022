package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util.LimelightV2;

public class TurretAiming extends CommandBase {
    protected LazySusanSubsystem lazySusanSubsystem;

    public TurretAiming(LazySusanSubsystem lazySusanSubsystem) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        addRequirements(lazySusanSubsystem);
    }

    @Override
    public void initialize() {
        LimelightV2.ledOn();
    }

    @Override
    public void execute() {
        double x = LimelightV2.tX();
        double speed = -(x-Constants.leftBias)*Constants.diffConstLS;

        if (!ShooterMath.inBounds(speed > 0, this.lazySusanSubsystem.getLazySusanPosition()))
            speed = 0;
        this.lazySusanSubsystem.setLazySusanSpeed(speed);
    }

    @Override
    public void end(boolean interrupt) {
        this.lazySusanSubsystem.setLazySusanSpeed(0);
        LimelightV2.ledOff();
    }
}