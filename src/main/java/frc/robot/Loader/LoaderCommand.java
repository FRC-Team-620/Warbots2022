package frc.robot.Loader;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LoaderCommand extends CommandBase {
    protected LoaderSubsystem loaderSubsystem;
    protected XboxController operatorXbox;
    protected boolean autoOn;
    protected boolean autoFire;
    

    public LoaderCommand(LoaderSubsystem loaderSubsystem, XboxController operatorXbox) {
        addRequirements(loaderSubsystem);
        this.loaderSubsystem = loaderSubsystem;
        this.operatorXbox = operatorXbox;
        autoOn = false;
        autoFire = false;
        // Use addRequirements() here to declare subsystem dependencies.
    }
    @Override
    public void execute() {
        CANSparkMax temp = loaderSubsystem.getLoaderMotor();
        if (operatorXbox.getLeftBumperPressed()) {
            if (temp.getEncoder().getVelocity() > 0) {
                temp.set(0);
            } else {
                temp.set(1);
            }
        } else if (autoOn) {
            if (temp.getEncoder().getVelocity() > 0) {
                temp.set(0);
                autoOn = false;
            } else {
                temp.set(1);
                autoOn = false;
            }
        }
        if (operatorXbox.getAButtonPressed()) {
            loaderSubsystem.getSolenoid().set(true);
            // autoFire = false;
            //  || autoFire == false
            //  || autoFire
        } else if (operatorXbox.getAButtonReleased()) {
            loaderSubsystem.getSolenoid().set(false);
        }
    }

    public void setAutoOn(boolean x) {
        autoOn = x;
    }
    public void setAutoFire(boolean x) {
        autoFire = x;
    }
}
