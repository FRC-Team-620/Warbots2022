package frc.robot.Drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Controls.ControlConstants;

public class DriveWithJoystick extends CommandBase {
    protected Drivetrain drivetrain;
    protected XboxController driverXbox;
    protected double speedConstant = Constants.speedHigh;
    protected double rotationConstant = Constants.rotationHigh;
    protected double rampRateConstant = Constants.rampRate;
    protected boolean squareInputsConstant = Constants.SQUARE_INPUTS;
    protected boolean isDriving = false;
    protected double fudgeFactor = 0.07;

    public DriveWithJoystick(Drivetrain drivetrain, XboxController driverXbox) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.driverXbox = driverXbox;

        SmartDashboard.putNumber("DriveWithJoystick/speed", speedConstant);
        SmartDashboard.putNumber("DriveWithJoystick/rotation", rotationConstant);
        SmartDashboard.putNumber("DriveWithJoystick/fudgeFactor", fudgeFactor);
        SmartDashboard.putNumber("DriveWithJoystick/rampRate", rampRateConstant);
        SmartDashboard.putBoolean("DriveWithJoystick/squareInputs", squareInputsConstant);
        // drivetrain.setAllOpenLoopRampRates(openLoopRampRateConstant);
    }

    @Override
    public void execute() {
        this.speedConstant = SmartDashboard.getNumber("DriveWithJoystick/speed", Constants.speedHigh);
        this.rotationConstant = SmartDashboard.getNumber("DriveWithJoystick/rotation", Constants.rotationHigh);
                this.rampRateConstant = SmartDashboard.getNumber("DriveWithJoystick/rampRate", Constants.rampRate);
        this.squareInputsConstant = SmartDashboard.getBoolean("DriveWithJoystick/squareInputs", Constants.SQUARE_INPUTS);
        this.fudgeFactor = SmartDashboard.getNumber("DriveWithJoystick/fudgeFactor", fudgeFactor);

        // Xbox Controller Input
        double rightTriggerInput = Math.pow(driverXbox.getRightTriggerAxis(), 2); // Forwards
        double leftTriggerInput = Math.pow(driverXbox.getLeftTriggerAxis(), 2); // Backwards
        double rotationInput = Math.pow(driverXbox.getLeftX(), squareInputsConstant ? 2 : 1);

        // Only apply the original sign if the factor is even
        if (squareInputsConstant) rotationInput *= Math.signum(driverXbox.getLeftX());

        double direction = rightTriggerInput > leftTriggerInput ? rightTriggerInput : -leftTriggerInput;
        double speed = direction * speedConstant;
        double rotation = rotationConstant * rotationInput;
        
        drivetrain.curvatureInput(speed, rotation, true); // Always enable turning in place
        if (speed > 0) drivetrain.setFudgeSpeed(fudgeFactor);
        else if (speed < 0) drivetrain.setFudgeSpeed(-fudgeFactor);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
