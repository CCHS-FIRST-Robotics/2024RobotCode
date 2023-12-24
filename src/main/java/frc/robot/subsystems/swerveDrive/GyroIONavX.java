package frc.robot.subsystems.swerveDrive;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

/** IO implementation for NavX */
public class GyroIONavX implements GyroIO {
    private final AHRS navx;

    public GyroIONavX() {
        System.out.println("[Init] Creating GyroIONavX");
        navx = new AHRS();
        navx.reset();
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.rollPosition = Degrees.of(navx.getRoll());
        inputs.pitchPosition = Degrees.of(navx.getPitch());
        inputs.yawPosition = Degrees.of(navx.getYaw());
        
        inputs.rollVelocity = DegreesPerSecond.of(navx.getRawGyroY());
        inputs.pitchVelocity = DegreesPerSecond.of(navx.getRawGyroX());
        inputs.yawVelocity = DegreesPerSecond.of(-navx.getRawGyroZ()); // negative cuz its switched for some reason
    }
}

