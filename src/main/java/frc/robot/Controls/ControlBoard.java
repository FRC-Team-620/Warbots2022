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
        lowShotButton,
        extendArms,
        retractArms;

    public static TriggerPressed aimTurretTrigger;

    // actual controllers
    private static XboxController driver;

    public static void init() {
        // controllers
        driver = new XboxController(0);
        ControlBoard.initDriverControls();
    }

    private static void initDriverControls() {
        // driver controls
        intakeButton = new JoystickButton(driver, Button.kA.value);
        lowShotButton = new JoystickButton(driver, Button.kRightBumper.value);
        extendArms = new JoystickButton(driver, Button.kX.value);
        retractArms = new JoystickButton(driver, Button.kY.value);
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

    // use these once drive command is converted
    public static double driveSpeedControl(boolean squaredInput) {
        double forward = driver.getRightTriggerAxis();
        double reverse = driver.getLeftTriggerAxis();
        double input = forward - reverse;
        input = squaredInput ? squareInput(input) : input;
        return input * ControlConstants.DRIVE_SPEED_PERCENT;
    }

    public static double driveRotationcontrol(boolean squaredInput) {
        double input = driver.getRightX();
        input = squaredInput ? squareInput(input) : input;
        return input * ControlConstants.DRIVE_ROTATION_PERCENT;
    }

    private static double squareInput(double input) {
        double negativePreserver = input > 0 ? 1 : -1;
        double squaredInput = Math.pow(input, ControlConstants.SQUARE_FACTOR);
        return negativePreserver * squaredInput;
    }
}
