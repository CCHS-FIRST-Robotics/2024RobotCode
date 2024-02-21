// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do:
 * 
 * shooter:
 * Proper feedforward and feedback using the Phoenix API for velocity control,
 * using exponential motion magic (check the Phoenix docs for details)
 * 
 * A sysid routine and signal logger export for motor data (should be pretty
 * much identical to how the arm was done, David you might want to do this one)
 * 
 * ! ask colin if intake and shoot commands should move the arm to angle or if
 * should be done manually
 */
public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
