package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Drive.Drivetrain;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimeLight.LedMode;

public class TankDriveAiming extends CommandBase {
    protected Drivetrain drivetrain;
    protected int frames, maxFrames;

    public TankDriveAiming(Drivetrain drivetrain, int maxFrames) {
        this.drivetrain = drivetrain;
        this.maxFrames = maxFrames;
        addRequirements(drivetrain);
    }
    public TankDriveAiming(Drivetrain drivetrain) {
        this(drivetrain, -1);
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
        double speed = -(x-Constants.leftBias)*Constants.diffConstTankDriveAim;
        System.out.println("Speed: " + speed);

        this.drivetrain.tankDriveSet(-speed, speed);
    }

    @Override
    public void end(boolean interrupt) {
        this.drivetrain.tankDriveSet(0, 0);
        LimeLight.setLedMode(LedMode.OFF);
    }

    @Override
    public boolean isFinished() {
        return this.maxFrames == -1 ? false : this.frames >= this.maxFrames;
    }
}

