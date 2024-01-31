package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.geometry.Rotation2d;

public class Drive extends SubsystemBase {

    private Module[] modules;
    private ChassisSpeeds speeds;
    private SwerveModuleState state;
    private Rotation2d angle;
    private double speedMetersPerSecond;
    

    public Drive() {
        speeds = new ChassisSpeeds();
        modules = new Module[4];
        angle = new Rotation2d(2);
        speedMetersPerSecond = 0;

        for (int i = 0; i < modules.length; i++) {
            modules[i] = new Module(i);
        }

        state = new SwerveModuleState(speedMetersPerSecond, angle);
    }

    public void periodic(){
    
        for (Module module : modules) {
            module.drive(state);
        }

    }
    
}
