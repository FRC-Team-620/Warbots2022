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
    protected Solenoid[] solenoids;

    protected int prevTime = -1;

    public ClimberCommand(ClimberSubsystem climberSubsystem, XboxController driverXbox) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
        this.driverXbox = driverXbox;
        this.solenoids = new Solenoid[] {
            new Solenoid(PneumaticsModuleType.CTREPCM, 0),
            new Solenoid(PneumaticsModuleType.CTREPCM, 1),
            new Solenoid(PneumaticsModuleType.CTREPCM, 2),
            new Solenoid(PneumaticsModuleType.CTREPCM, 3)
        };
    }

    @Override
    public void execute() {
        // Testing if it's working
        solenoids[0].set(driverXbox.getAButton());
        solenoids[1].set(driverXbox.getYButton());
        solenoids[2].set(driverXbox.getBButton());
        solenoids[3].set(driverXbox.getXButton());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
