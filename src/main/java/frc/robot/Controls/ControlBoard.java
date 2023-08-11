package frc.robot.Controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class ControlBoard {

    // driver
    public static JoystickButton 
        intakeButton,
        lowShotButton;

    // actual controllers
    private static XboxController driver;

    public static void init() {
        // controllers
        driver = new XboxController(0);
        ControlBoard.initDriverControls();
    }

    private static void initDriverControls() {
        // driver controls
        intakeButton = new JoystickButton(driver, Button.kLeftBumper.value);
        lowShotButton = new JoystickButton(driver, Button.kRightBumper.value);
    }

    // use these for now until the drive command is converted
    public static XboxController getDriverController() {
        return driver;
    }

    public static void setDriverRumble(boolean onOff) {
        setDriverLowFreqRumble(onOff);
        setDriverHighFreqRumble(onOff);
    }

    public static void setDriverLowFreqRumble(boolean onOff) {
        double rumble = onOff ? Constants.operatorRumbleLowFreq : 0;
        driver.setRumble(RumbleType.kLeftRumble, rumble);
    }

    public static void setDriverHighFreqRumble(boolean onOff) {
        double rumble = onOff ? Constants.operatorRumbleHighFreq : 0;
        driver.setRumble(RumbleType.kRightRumble, rumble);
    }
}
