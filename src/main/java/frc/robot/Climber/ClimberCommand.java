package frc.robot.Climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberCommand extends CommandBase {
    protected XboxController driverXbox;
    protected ClimberSubsystem climberSubsystem;

    protected int prevTime = -1;

    public ClimberCommand(ClimberSubsystem climberSubsystem, XboxController driverXbox) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
        this.driverXbox = driverXbox;
    }

    @Override
    public void execute() {
        // Testing if it's working
        climberSubsystem.climberSolenoid.set(driverXbox.getYButton());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
