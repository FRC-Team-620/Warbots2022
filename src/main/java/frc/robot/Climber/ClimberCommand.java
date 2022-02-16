package frc.robot.Climber;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberCommand extends CommandBase {
    protected XboxController driverXbox;
    protected ClimberSubsystem climberSubsystem;
    protected Solenoid climberSolenoid;

    protected int prevTime = -1;

    public ClimberCommand(ClimberSubsystem climberSubsystem, XboxController driverXbox) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
        this.driverXbox = driverXbox;
        this.climberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    }

    @Override
    public void execute() {
        // Testing if it's working
        climberSolenoid.set(driverXbox.getYButton());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
