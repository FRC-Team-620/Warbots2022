package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Drive.Drivetrain;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;

public class TankDriveAiming extends CommandBase {
    protected Drivetrain drivetrain;

    public TankDriveAiming(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        LimeLight.setLedMode(LedMode.ON);
    }

    @Override
    public void execute() {
        double x = LimeLight.getTX();
        double speed = -(x-Constants.leftBias)*Constants.diffConstTankDriveAim;
        System.out.println("Speed: " + speed);

        this.drivetrain.tankDriveSet(-speed, speed);
    }

    @Override
    public void end(boolean interrupt) {
        this.drivetrain.tankDriveSet(0, 0);
        LimeLight.setLedMode(LedMode.OFF);
    }
}

