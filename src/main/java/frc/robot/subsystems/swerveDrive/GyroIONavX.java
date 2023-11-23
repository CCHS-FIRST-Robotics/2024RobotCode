package frc.robot.subsystems.swerveDrive;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.math.util.Units;

/** IO implementation for NavX */
public class GyroIONavX implements GyroIO {
    private final AHRS navx;

    public GyroIONavX() {
        System.out.println("[Init] Creating GyroIONavX");
        navx = new AHRS();
        navx.calibrate();
        navx.reset();
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.rollPositionRad = Units.degreesToRadians(navx.getRoll());
        inputs.pitchPositionRad = Units.degreesToRadians(navx.getPitch());
        inputs.yawPositionRad = Units.degreesToRadians(navx.getYaw());
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(navx.getRawGyroY());
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(navx.getRawGyroX());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(navx.getRate());
    }
}

