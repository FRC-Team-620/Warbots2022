package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoShoot extends CommandBase {
    protected LoaderSubsystem loaderSubsystem;
    protected int frames = 0;
    public AutoShoot(LoaderSubsystem loaderSubsystem) {
        //addRequirements(loaderSubsystem);
        this.loaderSubsystem = loaderSubsystem;
    }

    @Override
    public void execute() {
        frames++;
    }

    @Override
    public void initialize() {
        this.frames = 0;
        System.out.println("Ball was shot");
        loaderSubsystem.getLoaderSolenoid().set(true);
    }

    @Override
    public void end(boolean interrupted) {
        loaderSubsystem.getLoaderSolenoid().set(false);
    }

    @Override
    public boolean isFinished() {
        return frames >= 80;
    }
}
