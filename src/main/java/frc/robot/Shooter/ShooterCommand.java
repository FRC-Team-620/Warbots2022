package frc.robot.Shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterCommand extends CommandBase {
    protected XboxController driverXbox;
    protected ShooterSubsystem shooterSubsystem;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, XboxController driverXbox) {
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.driverXbox = driverXbox;
    }

    @Override
    public void execute() {
        double speed = driverXbox.getRightY();
        shooterSubsystem.setShooterSpeed(speed);
    }
}
