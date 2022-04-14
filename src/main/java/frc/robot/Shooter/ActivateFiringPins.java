package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Loader.Intake;

public class ActivateFiringPins extends CommandBase {
    private FiringPins firingPins;
    private Intake intake;
    private int frames;

    public ActivateFiringPins(FiringPins firingPins, Intake intake) {
        addRequirements(intake);
        this.firingPins = firingPins;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        this.frames = 0;
        System.out.println("Ball was shot");
        this.intake.enableInnerIntakeMotor();
    }

    @Override
    public void execute() {
        // The below value needs to be tuned to be as fast as possible
        if (++this.frames >= 15)
            this.firingPins.extendFiringPinsSolenoid();
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.disableInnerIntakeMotor();;
        this.firingPins.retractFiringPinsSolenoid();
    }

    @Override
    public boolean isFinished() {
        return frames >= 80;
    }
}
