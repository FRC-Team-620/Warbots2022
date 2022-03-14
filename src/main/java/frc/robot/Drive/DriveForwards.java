package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForwards extends CommandBase {
    private Drivetrain drivetrain;
    private double frames;
    public DriveForwards(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    @Override
    public void initialize() {
        this.frames = 0;
    }

    @Override
    public void execute() {
        drivetrain.curvatureInput(0.5, 0, false);
        frames++;
    }

    @Override
    public boolean isFinished() {
        return frames > 40;
    }
}

