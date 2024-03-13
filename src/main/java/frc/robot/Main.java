// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do:
 * 
 * sysID for shooter (same as arm)
 * add a fixed arm angle for speaker so that we can still shoot if odom is
 * broken
 * method for base intake that doesn't autostop
 * start arm intake and then move arm down
 * when arm down, start base intake continuously until arm intake detects it
 */
public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}