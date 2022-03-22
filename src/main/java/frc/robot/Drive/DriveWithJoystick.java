package frc.robot.Drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Controls.ControlConstants;

public class DriveWithJoystick extends CommandBase {
    protected Drivetrain drivetrain;
    protected XboxController driverXbox;
    protected XboxController operatorXbox;
    protected double speedConstant = Constants.speedHigh;
    protected double rotationConstant = Constants.rotationHigh;
    protected double openLoopRampRateConstant = Constants.rampRate;
    protected boolean isDriving = false;

    public DriveWithJoystick(Drivetrain drivetrain, XboxController driverXbox, XboxController operatorXbox) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.driverXbox = driverXbox;
        this.operatorXbox = operatorXbox;

        SmartDashboard.putNumber("speed", speedConstant);
        SmartDashboard.putNumber("rotation", rotationConstant);
        SmartDashboard.putNumber("openloopramprate", openLoopRampRateConstant);
        drivetrain.setAllOpenLoopRampRates(openLoopRampRateConstant);
    }

    @Override
    public void execute() {
        if (driverXbox.getBButton()) {
            this.speedConstant = Constants.speedLow;
            this.rotationConstant = Constants.rotationLow;
        } else {
            this.speedConstant = Constants.speedHigh;
            this.rotationConstant = Constants.rotationHigh;
        }

        // Xbox Controller Input
        double rightTriggerInput = Math.pow(driverXbox.getRightTriggerAxis(), 2); // Forwards
        double leftTriggerInput = Math.pow(driverXbox.getLeftTriggerAxis(), 2); // Backwards
        double rotationInput = Math.pow(driverXbox.getLeftX(), ControlConstants.SQUARE_FACTOR);

        // Only apply the original sign if the factor is even
        if(ControlConstants.SQUARE_FACTOR % 2 == 0) 
            rotationInput *= Math.signum(driverXbox.getLeftX());

        double direction = rightTriggerInput > leftTriggerInput ? rightTriggerInput : -leftTriggerInput;
        double speed = direction * speedConstant;
        double rotation = rotationConstant * rotationInput;
        
        drivetrain.curvatureDrive(speed, rotation, true); // Always enable turning in place
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
