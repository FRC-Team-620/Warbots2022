// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import edu.wpi.first.wpilibj.XboxController;
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
    double rotation = Constants.rotation * driverXbox.getLeftX();
    double speed = 0.0;

    if (driverXbox.getRightTriggerAxis() > driverXbox.getLeftTriggerAxis()) {
      speed = driverXbox.getRightTriggerAxis() * Constants.speed;
    } else if (driverXbox.getRightTriggerAxis() < driverXbox.getLeftTriggerAxis()) {
      speed = driverXbox.getLeftTriggerAxis() * -Constants.speed;
    }
    
    drivetrain.curvatureInput(speed, rotation, !(driverXbox.getBButton()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
