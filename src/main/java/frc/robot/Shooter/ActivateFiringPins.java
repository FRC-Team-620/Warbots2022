package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ActivateFiringPins extends CommandBase {
    protected FiringPins firingPins;
    protected int frames = 0;
    public ActivateFiringPins(FiringPins firingPins) {
        //addRequirements(loaderSubsystem);
        this.firingPins = firingPins;
    }

    @Override
    public void execute() {
        frames++;
    }

    @Override
    public void initialize() {
        this.frames = 0;
        System.out.println("Ball was shot");
        firingPins.extendFiringPinsSolenoid();
    }

    @Override
    public void end(boolean interrupted) {
        firingPins.retractFiringPinsSolenoid();
    }

    @Override
    public boolean isFinished() {
        return frames >= 80;
    }
}
