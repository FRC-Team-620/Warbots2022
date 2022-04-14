// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.DriveForwardDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Loader.Intake;

public class Taxi extends SequentialCommandGroup {
  /** Creates a new Taxi. */
  Drivetrain drivetrain;
  Intake intake;
  private double taxiDistanceMeters = 2;

  public Taxi(Drivetrain drivetrain, Intake intake) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    addCommands(
        new DriveForwardDistance(drivetrain, taxiDistanceMeters, this.intake));
  }

}
