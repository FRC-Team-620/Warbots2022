// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Drive.DriveWithJoystick;
import frc.robot.Drive.Drivetrain;
import frc.robot.Drive.RotateMotor;

/** Add your docs here. */
public class RobotContainer {
    protected Drivetrain drivetrain = new Drivetrain();
    protected XboxController driver = new XboxController(0);

    protected DriveWithJoystick driveWithJoystick;
    protected RotateMotor rotateMotor;

    // public Command getDriveWithJoystick() {
    // return new DriveWithJoystick(drivetrain, driver);
    // }

    public void init() {
        // rotateMotor = new RotateMotor(drivetrain, driver);
        driveWithJoystick = new DriveWithJoystick(drivetrain, driver);
        drivetrain.setDefaultCommand(driveWithJoystick);
        rotateMotor = new RotateMotor(drivetrain, driver);
        JoystickButton leftBumper = new JoystickButton(driver, Button.kLeftBumper.value);
        leftBumper.whenPressed(rotateMotor);
        // drivetrain.setDefaultCommand(rotateMotor);
        // rotateMotor.schedule(false);
    }

}