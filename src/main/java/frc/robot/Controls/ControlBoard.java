package frc.robot.Controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class ControlBoard {

    // driver
    public static JoystickButton intakeButton,
            outakeButton;

    // operator
    public static JoystickButton extendArmsButton,
            raiseArmsButton,
            climbSequenceButton,
            lowerHooksButton,
            tankDriveAimButton,
            lowShotButton,
            winchHoldButton,
            toggleGyroButton,
            reverseShooterWheelsButton;

    public static TriggerPressed aimTurretTrigger,
            fireTurretTrigger;

    // actual controllers
    private static XboxController driver, operator;

    public static void init() {
        // controllers
        driver = new XboxController(0);
        operator = new XboxController(1);

        ControlBoard.initDriverControls();
        ControlBoard.initOperatorControls();
    }

    private static void initDriverControls() {
        // driver controls
        intakeButton = new JoystickButton(driver, Button.kB.value);
        outakeButton = new JoystickButton(driver, Button.kA.value);
        winchHoldButton = new JoystickButton(driver, Button.kX.value);
    }

    private static void initOperatorControls() {
        // operator controls
        extendArmsButton = new JoystickButton(operator, Button.kX.value);
        raiseArmsButton = new JoystickButton(operator, Button.kY.value);
        reverseShooterWheelsButton = new JoystickButton(operator, Button.kA.value);
        climbSequenceButton = new JoystickButton(operator, Button.kB.value);

        aimTurretTrigger = new TriggerPressed(operator, Axis.kLeftTrigger.value);
        tankDriveAimButton = new JoystickButton(operator, Button.kLeftBumper.value);
        fireTurretTrigger = new TriggerPressed(operator, Axis.kRightTrigger.value);
        lowShotButton = new JoystickButton(operator, Button.kRightBumper.value);

        lowerHooksButton = new JoystickButton(operator, Button.kStart.value);
        toggleGyroButton = new JoystickButton(operator, Button.kBack.value);

        
    }

    // use these for now until the drive command is converted
    public static XboxController getDriverController() {
        return driver;
    }

    public static XboxController getOperatorController() {
        return operator;
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

    public static void setOperatorRumble(boolean onOff) {
        setOperatorLowFreqRumble(onOff);
        setOperatorHighFreqRumble(onOff);
    }

    public static void setOperatorLowFreqRumble(boolean onOff) {
        double rumble = onOff ? Constants.driverRumbleLowFreq : 0;
        operator.setRumble(RumbleType.kLeftRumble, rumble);
    }

    public static void setOperatorHighFreqRumble(boolean onOff) {
        double rumble = onOff ? Constants.driverRumbleHighFreq : 0;
        operator.setRumble(RumbleType.kRightRumble, rumble);
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
