// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.SPI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // TODO: Should this be its own file?
  public static class HardwareConstants {
    // the port for the xbox controller
    public static final int XBOX_CONTROLLER_PORT 			= 0;
    public static final int XBOX_CONTROLLER_ALTERNATE_PORT 	= 1;

      // for mecanum drive FR = front right, FL = front left, RR = rear right, RL = rear left
    public static final int FR_TALON_ID = 1;
    public static final int FL_TALON_ID = 2; 
    public static final int RR_TALON_ID = 3; 
    public static final int RL_TALON_ID = 4; 

    public static final int TALON_FX_CPR = 2048;
    // gear ratio of the motor gearbox
    public static final double FALCON_GEARBOX_RATIO = 10.71;

    public static final double MECANUM_WHEEL_DIAMETER 		= 0.1524; // meters

    public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;
    public static final SPI.Port ANALOG_GYRO_PORT = SPI.Port.kOnboardCS0;
    // public static final int ANALOG_GYRO_PORT = 0;

      // Robot's kinematics --> cartesian location of each wheel to the physical center of the robot in meters 
    public static final double WHEEL_ABSOLUTE_X_METERS = 0.2794;
    public static final double WHEEL_ABSOLUTE_Y_METERS = 0.31115;
    
    // public static final Translation2d FL_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_Y_METERS, WHEEL_ABSOLUTE_X_METERS);
    // public static final Translation2d FR_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_Y_METERS, WHEEL_ABSOLUTE_X_METERS);
    // public static final Translation2d RL_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_Y_METERS, -WHEEL_ABSOLUTE_X_METERS);
    // public static final Translation2d RR_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_Y_METERS, -WHEEL_ABSOLUTE_X_METERS);

    public static final Translation2d FL_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_X_METERS, WHEEL_ABSOLUTE_Y_METERS);
    public static final Translation2d FR_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_X_METERS, -WHEEL_ABSOLUTE_Y_METERS);
    public static final Translation2d RL_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_X_METERS, WHEEL_ABSOLUTE_Y_METERS);
    public static final Translation2d RR_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_X_METERS, -WHEEL_ABSOLUTE_Y_METERS);

    public static final MecanumDriveKinematics MECANUM_KINEMATICS = new MecanumDriveKinematics(FL_WHEEL_POS, FR_WHEEL_POS, RL_WHEEL_POS, RR_WHEEL_POS);
}
}
