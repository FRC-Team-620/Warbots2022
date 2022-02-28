// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Presets;

public class DriveWithJoystick extends CommandBase {
  protected Drivetrain drivetrain;
  protected XboxController driverXbox;
  protected double speedConstant = Constants.speed;
  protected double rotationConstant = Constants.rotation;
  protected double openLoopRampRateConstant = Constants.rampRate;
  protected SendableChooser<String> presetChooser = new SendableChooser<>();
  protected String currentDriver = "Default";
  protected List<Double> currentDriverPreset = Presets.presets.get(currentDriver);
  protected boolean isDriving = false;
  
  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick(Drivetrain drivetrain, XboxController driverXbox) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.driverXbox = driverXbox;

    SmartDashboard.putNumber("speed", currentDriverPreset.get(0));
    SmartDashboard.putNumber("rotation", currentDriverPreset.get(1));
    SmartDashboard.putNumber("openloopramprate", currentDriverPreset.get(2));
    presetChooser.setDefaultOption("Default", "Default");

    for (String preset : Presets.presets.keySet()) {
      if (!preset.toLowerCase().equals("default")) {
        presetChooser.addOption(preset, preset);
      }
    }

    SmartDashboard.putData("driver", presetChooser);    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // XBox controller input
    double rotationInput = (driverXbox.getLeftX() > 0 ? 1 : -1) * Math.pow(driverXbox.getLeftX(), 2);
    double rightTriggerInput = Math.pow(driverXbox.getRightTriggerAxis(), 2);
    double leftTriggerInput = Math.pow(driverXbox.getLeftTriggerAxis(), 2);

    double rotation = 0; //Constants.rotation = -0.5
    double speed = 0.0;
    openLoopRampRateConstant = drivetrain.getOpenLoopRampRate();
    
    NetworkTableEntry speedEntry = drivetrain.table.getEntry("speed");
    NetworkTableEntry rotationEntry = drivetrain.table.getEntry("rotation");
    NetworkTableEntry openLoopRampRateEntry = drivetrain.table.getEntry("openloopramprate");


    currentDriver = presetChooser.getSelected();
    currentDriverPreset = Presets.presets.get(currentDriver);
    
    // When the default drive mode is selected, then allow customizable values.
    if (currentDriver.toLowerCase().equals("default")) {
      //TESTING ONLY. SET BACK TO CONSTANTS FOR THE COMPETITION
      speedConstant = speedEntry.getDouble(0.5);
      rotationConstant = rotationEntry.getDouble(-0.5);
      openLoopRampRateConstant = openLoopRampRateEntry.getDouble(0.2);

    // Otherwise, use the driver-specific custom presets.
    } else {
      speedConstant = currentDriverPreset.get(0);
      speedEntry.setDouble(speedConstant);
      rotationConstant = currentDriverPreset.get(1);
      rotationEntry.setDouble(rotationConstant);
      openLoopRampRateConstant = currentDriverPreset.get(2);
      openLoopRampRateEntry.setDouble(openLoopRampRateConstant);
    }    
    
    drivetrain.setOpenLoopRampRate(openLoopRampRateConstant);

    rotation = rotationConstant * rotationInput;

    if (rightTriggerInput > leftTriggerInput) {
      speed = rightTriggerInput * speedConstant; // Constants.speed
    } else if (rightTriggerInput < leftTriggerInput) {
      speed = leftTriggerInput * -speedConstant;
    }

    if(rightTriggerInput == 0 && leftTriggerInput == 0 && rotationInput == 0) {
      if(isDriving) {
        for(int i = 1; i < 5; i++)
          drivetrain.setEncoderPos(i, 0);
        isDriving = false;
      }
      for(int i = 1; i < 5; i++)
        drivetrain.motorDrive(i, Constants.diffConstKeepPosition*drivetrain.getEncoderPos(i));
    } else {
      isDriving = true;
      drivetrain.curvatureInput(speed, rotation, !(driverXbox.getBButton()));
    }

    // SmartDashboard.putNumber("RightRPM",
    //     (drivetrain.getRPM(1) + drivetrain.getRPM(2)) / 2);
    // SmartDashboard.putNumber("LeftRPM",
    //     (drivetrain.getRPM(3) + drivetrain.getRPM(4)) / 2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
