package frc.robot.Loader;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LoaderCommand extends CommandBase {
    protected LoaderSubsystem loaderSubsystem;
    protected XboxController driverXbox;

    public LoaderCommand(LoaderSubsystem loaderSubsystem, XboxController driverXbox) {
        addRequirements(loaderSubsystem);
        this.loaderSubsystem = loaderSubsystem;
        this.driverXbox = driverXbox;
        // Use addRequirements() here to declare subsystem dependencies.
    }
    @Override
    public void execute() {
        CANSparkMax temp = loaderSubsystem.getLoaderMotor();
        if (temp.getEncoder().getVelocity() > 0) {
            temp.set(0);
        } else {
            temp.set(0.5);
        }
        
    }
}
