// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightV2
{
  static NetworkTable limelightTable;
  static NetworkTableEntry tx;
  static NetworkTableEntry ty;
  static NetworkTableEntry tv;
  static NetworkTableEntry ledMode;

  private static final int ledOn = 3;
  private static final int ledOff = 1;
  
  public static void init() 
  {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    tv = limelightTable.getEntry("tv");
    ledMode = limelightTable.getEntry("ledMode");
  }

  public static void ledOn()
  {
    ledMode.setNumber(ledOn);
  }
  
  public static void ledOff()
  {
    ledMode.setNumber(ledOff);
  }

  public static double tX()
  {
    return tx.getDouble(0);
  }

  public static double tY()
  {
    return ty.getDouble(0);
  }
  
  public static boolean targetFound()
  {
    if ((int)tv.getNumber(0) == 1)
    {
      return true;
    }
    
    return false;
  }
}
