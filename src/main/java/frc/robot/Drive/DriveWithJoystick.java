// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Presets;

public class DriveWithJoystick extends CommandBase {
  protected Drivetrain drivetrain;
  protected XboxController driverXbox;
  protected XboxController operatorXbox;
  protected double speedConstant = Constants.speedHigh;
  protected double rotationConstant = Constants.rotationHigh;
  protected double openLoopRampRateConstant = Constants.rampRate;
  protected double operatorControlCoef = 0.15;
  protected SendableChooser<String> presetChooser = new SendableChooser<>();
  protected String currentDriver = "Default";
  protected List<Double> currentDriverPreset = Presets.presets.get(currentDriver);
  protected boolean isDriving = false;

  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick(Drivetrain drivetrain, XboxController driverXbox, XboxController operatorXbox) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.driverXbox = driverXbox;
    this.operatorXbox = operatorXbox;

    SmartDashboard.putNumber("speed", currentDriverPreset.get(0));
    SmartDashboard.putNumber("rotation", currentDriverPreset.get(1));
    SmartDashboard.putNumber("openloopramprate", currentDriverPreset.get(2));
    presetChooser.setDefaultOption("Default", "Default");

    for (String preset : Presets.presets.keySet()) {
      if (!preset.toLowerCase().equals("default")) {
        presetChooser.addOption(preset, preset);
      }
    }

    SmartDashboard.putData("driver", presetChooser); // Use addRequirements() here to declare subsystem dependencies.
    drivetrain.setOpenLoopRampRate(openLoopRampRateConstant);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driverXbox.getBButton()) {
      speedConstant = Constants.speedLow;
      rotationConstant = Constants.rotationLow;
    } else {
      speedConstant = Constants.speedHigh;
      rotationConstant = Constants.rotationHigh;
    }
    openLoopRampRateConstant = Constants.rampRate;

    // XBox controller input
    // get rotation input
    double rotationInput = (driverXbox.getLeftX() > 0 ? 1 : -1) * Math.pow(driverXbox.getLeftX(), 2);
    // accelerate
    double rightTriggerInput = Math.pow(driverXbox.getRightTriggerAxis(), 2);
    // decellerate
    double leftTriggerInput = Math.pow(driverXbox.getLeftTriggerAxis(), 2);

    double rotation = 0; // Constants.rotation = -0.5
    double speed = 0.0;
    openLoopRampRateConstant = drivetrain.getOpenLoopRampRate();

    rotation = rotationConstant * rotationInput;

    if (rightTriggerInput > leftTriggerInput) {
      speed = rightTriggerInput * speedConstant; // Constants.speed
    } else if (rightTriggerInput < leftTriggerInput) {
      speed = leftTriggerInput * -speedConstant;
    }
    
    drivetrain.curvatureInput(speed, rotation, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
