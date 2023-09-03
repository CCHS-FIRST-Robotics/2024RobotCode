package frc.robot.subsystems.mecaDrive;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.DCMotor;

public class MecanumDrivetrainSim {
    FlywheelSim frMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    FlywheelSim flMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    FlywheelSim brMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    FlywheelSim blMotor = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
}
