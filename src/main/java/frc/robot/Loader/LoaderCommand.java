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
        CANSparkMax loaderMotor = loaderSubsystem.getLoaderMotor();
        // reverse intake
        if (!driverXbox.getBButton()) {
            if (driverXbox.getAButton()) {
                this.loaderSubsystem.retractExtensionSolenoid();
                this.loaderSubsystem.reverseLoaderMotor();
            } else {
                this.loaderSubsystem.disableLoaderMotor();
            }
        }
        // intake
        if (!driverXbox.getAButton()) {
            if (driverXbox.getBButton()) {
                this.loaderSubsystem.enableLoaderMotor();
                this.loaderSubsystem.extendExtensionSolenoid();;
                this.loaderSubsystem.enableExtendedLoaderMotor();
            } else {
                this.loaderSubsystem.disableLoaderMotor();
                this.loaderSubsystem.retractExtensionSolenoid();
                if (!loaderSubsystem.isClimbing()) {
                    this.loaderSubsystem.retractExtensionSolenoid();
                }
                
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
