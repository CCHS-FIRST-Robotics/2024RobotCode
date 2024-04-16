// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do:
 * 
 * at robo:
 * tune module PID
 * FOC instead of MotionMagic armIOFalcon
 * 
 * 
 * small things:
 * make objects in IO classes final
 * moduleIO kV and kA
 * GyroIONavX negative values
 * 
 * 
 * ask colin:
 * find out where the gyro is actually used
 * fix something in drivewithjoysticks
 * something I don't understand in gyroIO.java
 * 
 * 
 * eventually:
 * autochooser for autos
 * try to get advantagekit to simulate the entire robot
 * don't use timestampedpose2d/3d
 * tunable numbers with smartdashboard
 * phoenix sims
 */

public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}