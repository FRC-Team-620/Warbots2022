// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.DriveForwardDistance;
import frc.robot.Drive.Drivetrain;

public class Taxi extends SequentialCommandGroup {
  /** Creates a new Taxi. */
  Drivetrain drivetrain;
  private double taxiDistanceMeters = 2;

  public Taxi(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addCommands(
        new DriveForwardDistance(drivetrain, taxiDistanceMeters));
  }

}
