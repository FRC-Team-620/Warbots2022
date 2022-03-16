package frc.robot.Controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ControlBoard {

    // driver
    public JoystickButton intakeButton,
            outakeButton;

    // operator
    public JoystickButton extendArmsButton,
            raiseArmsButton,
            climbSequenceButton,
            lowerHooksButton,
            retractArmsButton,
            lowShotButton,
            winchHoldButton;

    public TriggerPressed aimTurretTrigger,
            fireTurretTrigger;

    // actual controllers
    private XboxController driver,
            operator;

    public ControlBoard() {
        // controllers
        driver = new XboxController(0);
        operator = new XboxController(1);

        initDriverControls();
        initOperatorControls();

    }

    private void initDriverControls() {
        // driver controls
        intakeButton = new JoystickButton(driver, Button.kB.value);
        outakeButton = new JoystickButton(driver, Button.kA.value);
        lowShotButton = new JoystickButton(driver, Button.kY.value);
        winchHoldButton = new JoystickButton(driver, Button.kX.value);
    }

    private void initOperatorControls() {
        // operator controls
        extendArmsButton = new JoystickButton(operator, Button.kX.value);
        raiseArmsButton = new JoystickButton(operator, Button.kY.value);
        climbSequenceButton = new JoystickButton(operator, Button.kB.value);
        lowerHooksButton = new JoystickButton(operator, Button.kStart.value);
        retractArmsButton = new JoystickButton(operator, Button.kLeftBumper.value);
        aimTurretTrigger = new TriggerPressed(operator, Axis.kLeftTrigger.value);
        fireTurretTrigger = new TriggerPressed(operator, Axis.kRightTrigger.value);
    }

    // use these for now until the drive command is converted
    public XboxController getDriverController() {
        return driver;
    }

    public XboxController getOperatorController() {
        return operator;
    }

    // use these once drive command is converted
    public double driveSpeedControl(boolean squaredInput) {
        double forward = driver.getRightTriggerAxis();
        double reverse = driver.getLeftTriggerAxis();
        double input = forward - reverse;
        input = squaredInput ? squareInput(input) : input;
        return input * ControlConstants.DRIVE_SPEED_PERCENT;
    }

    public double driveRotationcontrol(boolean squaredInput) {
        double input = driver.getRightX();
        input = squaredInput ? squareInput(input) : input;
        return input * ControlConstants.DRIVE_ROTATION_PERCENT;
    }

    private double squareInput(double input) {
        double negativePreserver = input > 0 ? 1 : -1;
        double squaredInput = Math.pow(input, ControlConstants.SQUARE_FACTOR);
        return negativePreserver * squaredInput;
    }

}
