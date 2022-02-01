// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class DriveWithJoystick extends CommandBase {
  protected Drivetrain drivetrain;
  protected XboxController driverXbox;

  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick(Drivetrain drivetrain, XboxController driverXbox) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.driverXbox = driverXbox;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // XBox controller input
    double rotationInput = (driverXbox.getLeftX() > 0 ? 1 : -1) * Math.pow(driverXbox.getLeftX(), 2);
    double rightTriggerInput = Math.pow(driverXbox.getRightTriggerAxis(), 2);
    double leftTriggerInput = Math.pow(driverXbox.getLeftTriggerAxis(), 2);

    double rotation = Constants.rotation * rotationInput;
    double speed = 0.0;

    if (rightTriggerInput > leftTriggerInput) {
      speed = rightTriggerInput * Constants.speed;
    } else if (rightTriggerInput < leftTriggerInput) {
      speed = leftTriggerInput * -Constants.speed;
    }

    SmartDashboard.putNumber("Right RPM: ",
        (drivetrain.getRPM(1) + drivetrain.getRPM(2)) / 2);
    SmartDashboard.putNumber("Left RPM: ",
        (drivetrain.getRPM(3) + drivetrain.getRPM(4)) / 2);

    drivetrain.curvatureInput(speed, rotation, !(driverXbox.getBButton()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
