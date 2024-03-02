// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do:
 * 
 * better simulation code
 * 
 * sysID for shooter (same as arm)
 * 
 * magic number cleanup
 * 
 * We also need to fix the velocity control currently, it uses WPILib's
 * PID and FF classes, but it should be using the Phoenix API to control
 * velocity. It should use setControl() like the arm code, and the
 * configs need to be setup properly.
 * 
 */
public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}