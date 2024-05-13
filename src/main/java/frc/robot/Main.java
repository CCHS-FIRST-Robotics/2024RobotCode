// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do:
 * hunt for magic numbers
 * alex needs to figure out more levels for the code hierarchy
 * moduleIO kV and kA
 * mayyyyybe javadoc (probably not)
 * 
 * at robo:
 * sysID
 * FOC instead of MotionMagic in armIOFalcon
 * 
 * ask colin:
 * something in gyroIO
 * what is fused in gyroIO used for (unused in NavX)
 * what is covar in cameraIO
 * 
 * eventually:
 * autochooser for autos
 * structs and protobuffs
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