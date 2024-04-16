// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Things to do next year:
 * 
 * tunable numbers with smartdashboard
 * phoenix sims
 * don't use timestampedpose2d/3d
 */

public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}