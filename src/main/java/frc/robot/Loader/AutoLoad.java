package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoLoad extends CommandBase{
    protected LoaderSubsystem loaderSubsystem;
    protected int frames = 0;
    protected int speed;
    public AutoLoad(LoaderSubsystem loaderSubsystem, int speed) {
        addRequirements(loaderSubsystem);
        this.loaderSubsystem = loaderSubsystem;
        this.speed = speed;
    }

    @Override
    public void execute() {
        frames++;
    }

    @Override
    public void initialize() {
        //System.out.println("Loader was turned on");
        loaderSubsystem.getLoaderMotor().set(speed);
    }
    
    @Override
    public void end(boolean interrupt) {
        loaderSubsystem.getLoaderMotor().set(0);
    }

    @Override
    public boolean isFinished() {
        return frames > 750;
    }
}
