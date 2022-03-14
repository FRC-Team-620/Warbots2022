package frc.robot.Loader;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LoaderCommand extends CommandBase {
    protected LoaderSubsystem loaderSubsystem;
    protected XboxController driverXbox;
    protected XboxController operatorXbox;
    protected boolean autoOn;
    protected boolean autoFire;
    

    public LoaderCommand(LoaderSubsystem loaderSubsystem, XboxController driverXbox, XboxController operatorXbox) {
        addRequirements(loaderSubsystem);
        this.loaderSubsystem = loaderSubsystem;
        this.driverXbox = driverXbox;
        this.operatorXbox = operatorXbox;
        autoOn = false;
        autoFire = false;
        // Use addRequirements() here to declare subsystem dependencies.
    }
    @Override
    public void execute() {
        CANSparkMax temp = loaderSubsystem.getLoaderMotor();
        //intake
        if (!driverXbox.getBButton()) {
            temp.setInverted(false);
            if (driverXbox.getAButton()) {
                temp.set(1);
            } else {
                temp.set(0);
            }
        }
        //reverse intake
        if (!driverXbox.getAButton()) {
            if (driverXbox.getBButton()) {
                temp.setInverted(true);
                temp.set(1);
            } else {
                temp.setInverted(false);
                temp.set(0);
            }
        }
        
        // if (autoOn) {
        //     temp.set(1);
        //     autoOn = false;
        // }
        
        // shooting code
        if (Math.abs(operatorXbox.getRightTriggerAxis()) > 0) {
            loaderSubsystem.getLoaderSolenoid().set(true);
            // autoFire = false;
            //  || autoFire == false
            //  || autoFire
        } else if (Math.abs(operatorXbox.getRightTriggerAxis()) == 0) {
            loaderSubsystem.getLoaderSolenoid().set(false);
        }
    }

    public void setAutoOn(boolean x) {
        autoOn = x;
    }
    public void setAutoFire(boolean x) {
        autoFire = x;
    }
}
