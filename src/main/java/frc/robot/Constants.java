// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * PSEUDOCODE
 * 
 * Class Constants:
  
  Stores constants related to the controller/operator input
  Class OperatorConstants:
    
    Controller is connected to port 0 on the Driver Station
    Define constant integer kDriverControllerPort = 0

  Stores constants related to the robot's drive system
  Class DriveConstants:
    
  (Currently empty - will add motor IDs, gear ratios, etc. here when needed)
 */

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // TODO: Insert DriveConstants here...
  public static class DriveConstants {
  }
}
