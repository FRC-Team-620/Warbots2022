package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoLoad extends CommandBase{
    protected LoaderSubsystem loaderSubsystem;
    protected int frames = 0;
    public AutoLoad(LoaderSubsystem loaderSubsystem) {
        addRequirements(loaderSubsystem);
        this.loaderSubsystem = loaderSubsystem;
    }

    @Override
    public void execute() {
        frames++;
    }

    @Override
    public void initialize() {
        this.frames = 0;
        //System.out.println("Loader was turned on");
        loaderSubsystem.enableLoaderMotor();
        loaderSubsystem.enableExtendedLoaderMotor();
        loaderSubsystem.extendExtensionSolenoid();
    }
    
    @Override
    public void end(boolean interrupt) {
        loaderSubsystem.disableLoaderMotor();
        loaderSubsystem.disableExtendedLoaderMotor();
        loaderSubsystem.retractExtensionSolenoid();
    }

    @Override
    public boolean isFinished() {
        
        return frames > 750;
    }
}
