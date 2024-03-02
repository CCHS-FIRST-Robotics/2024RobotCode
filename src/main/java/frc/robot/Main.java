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
 * The current detection stuff needs to be cleaned up a bit,
 * particularly on the shooter side. We have a few magic numbers in there,
 * some old methods, and the logic of some of it could be changed.
 * 
 * We also need to fix the velocity control currently, it uses WPILib's
 * PID and FF classes, but it should be using the Phoenix API to control
 * velocity. It should use setControl() like the arm code, and the
 * configs need to be setup properly.
 * 
 * We should also be logging data from both motors, not just one. The motors are
 * independent of one another (mechanically), so they should be separate, and
 * the position/velocity (and definitely the rest) of each should be
 * logged.
 */
public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}