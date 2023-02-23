// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Routines;

import javax.swing.UIDefaults.LazyInputMap;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auto.DriveForwardDistance;
import frc.robot.Drive.Drivetrain;
import frc.robot.Loader.Intake;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.ZeroTurnTable;

public class Taxi extends ParallelCommandGroup {
  /** Creates a new Taxi. */
  Drivetrain drivetrain;
  Intake intake;
  LazySusanSubsystem lazySusanSubsystem;
  private double taxiDistanceMeters = 2;

  public Taxi(Drivetrain drivetrain, Intake intake, LazySusanSubsystem lazySusanSubsystem) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.lazySusanSubsystem = lazySusanSubsystem;
    if (!lazySusanSubsystem.getIsCal()) {
      addCommands(
        new ZeroTurnTable(lazySusanSubsystem),
        new DriveForwardDistance(drivetrain, taxiDistanceMeters, this.intake));
    } else {
      addCommands(
        new DriveForwardDistance(drivetrain, taxiDistanceMeters, this.intake));
    }

    
  }

}
