package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Drive.Drivetrain;
import frc.robot.Util.LimelightV2;

public class TankDriveAiming extends CommandBase {
    protected Drivetrain drivetrain;

    public TankDriveAiming(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        LimelightV2.ledOn();
    }

    @Override
    public void execute() {
        double x = LimelightV2.tX();
        double speed = -(x-Constants.leftBias)*Constants.diffConstTankDriveAim;

        this.drivetrain.tankDriveSet(-speed, speed);
    }

    @Override
    public void end(boolean interrupt) {
        this.drivetrain.tankDriveSet(0, 0);
        LimelightV2.ledOff();
    }
}

