// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * to do:
 * 
 * arm stuff
 * for example actually writing a controlwithJoysticks command
 * ^ thats not really necessary tbh, check my texts in the gc with david
 * 
 * shooter:
 * Proper feedforward and feedback using the Phoenix API for velocity control,
 * using exponential motion magic (check the Phoenix docs for details) <--- not doing this anymore, just regular VelocityVoltage control
 * A sysid routine and signal logger export for motor data (should be pretty
 * much identical to how the arm was done, David you might want to do this one)
 * 
 * ! ask colin if intake and shoot commands should move the arm to angle (yes)
 * ! ask colin about conflicting button mappings in robotcontainer (fix them)
 * ! ask colin why shooter even needs motionmagic (it doesn't)
 * 
 * ! ask colin if the neos for arm intake will have encoders (NEOs are brushless motors, so yes)
 * ! shooter motion magic stuff (see above)
 * ! intake shoot() command (see comments)
 */
public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
