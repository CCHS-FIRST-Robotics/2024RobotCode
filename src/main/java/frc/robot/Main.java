// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do:
 * 
 * tune the PID in moduleSparkMax
 * 
 * tunable numbers with smartdashboard
 * moduleIO kV and kA
 * phoenix sims
 * don't use timestampedpose2d/3d
 * 
 * // ! do we need any raw acc data? I don't think we have any use for it
 * // ! dual imu??? https://arxiv.org/pdf/2107.02632.pdf in gyroIO.java
 * GyroIONavX negative values
 * 
 * make objects in IO classes final
 * find out where the gyro is actually used
 */

public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}