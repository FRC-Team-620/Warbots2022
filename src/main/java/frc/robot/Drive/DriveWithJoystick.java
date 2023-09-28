package frc.robot.Drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class DriveWithJoystick extends CommandBase {
    protected Drivetrain drivetrain;
    protected XboxController driverXbox;
    protected double speedConstant = Constants.speedHigh;
    protected double rotationConstant = Constants.rotationHigh;
    protected double rampRateConstant = Constants.rampRate;
    protected boolean squareInputsConstant = Constants.SQUARE_INPUTS;
    protected boolean isDriving = false;

    public DriveWithJoystick(Drivetrain drivetrain, XboxController driverXbox) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.driverXbox = driverXbox;

        SmartDashboard.putNumber("DriveWithJoystick/speed", speedConstant);
        SmartDashboard.putNumber("DriveWithJoystick/rotation", rotationConstant);
        SmartDashboard.putNumber("DriveWithJoystick/rampRate", rampRateConstant);
        SmartDashboard.putBoolean("DriveWithJoystick/squareInputs", squareInputsConstant);
        // drivetrain.setAllOpenLoopRampRates(openLoopRampRateConstant);
    }

    @Override
    public void execute() {
        // this.speedConstant = SmartDashboard.getNumber("DriveWithJoystick/speed", Constants.speedHigh);
        // this.rotationConstant = SmartDashboard.getNumber("DriveWithJoystick/rotation", Constants.rotationHigh);
        // this.rampRateConstant = SmartDashboard.getNumber("DriveWithJoystick/rampRate", Constants.rampRate);
        // this.squareInputsConstant = SmartDashboard.getBoolean("DriveWithJoystick/squareInputs", Constants.SQUARE_INPUTS);

        this.speedConstant = Constants.speedHigh;
        this.rotationConstant = Constants.rotationHigh;
        this.rampRateConstant = Constants.rampRate;
        this.squareInputsConstant = true;
        // Xbox Controller Input
        double rightTriggerInput = Math.pow(driverXbox.getRightTriggerAxis(), 2); // Forwards
        double leftTriggerInput = Math.pow(driverXbox.getLeftTriggerAxis(), 2); // Backwards
        double rotationInput = Math.pow(driverXbox.getLeftX(), squareInputsConstant ? 2 : 1);

        // Only apply the original sign if the factor is even
        if (squareInputsConstant) rotationInput *= Math.signum(driverXbox.getLeftX());

        double direction = rightTriggerInput - leftTriggerInput;
        double speed = direction * speedConstant;
        double rotation = rotationConstant * rotationInput;
        drivetrain.tankDriveSet(speed, 0);
        drivetrain.curvatureInput(speed, -rotation, true); // Always enable turning in place
        // double speed = driverXbox.getRightTriggerAxis();
        // drivetrain.tankDriveSet(speed, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
