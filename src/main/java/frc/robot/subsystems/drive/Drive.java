package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.proto.SwerveModuleStateProto;
import edu.wpi.first.math.geometry.Rotation2d;

public class Drive extends SubsystemBase {

    private Module[] modules;
    private ChassisSpeeds speeds;
    private SwerveModuleState state;
    private Rotation2d angle;
    private double speedMetersPerSecond;
    private double maxLinearSpeed; // should these use Measure??
    private double maxAngularSpeed;

    private double WHEEL_RADIUS = .0508; // fix where constants are

    public Drive() {
        speeds = new ChassisSpeeds();
        angle = new Rotation2d(0);
        speedMetersPerSecond = 0;

        maxLinearSpeed = 4.6;
        maxAngularSpeed = maxLinearSpeed / WHEEL_RADIUS; //not the wheel

        modules = new Module[4];
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new Module(i);
        }

        state = new SwerveModuleState();
    }

    public void periodic(){
        for (Module module : modules) {
            module.setReferences(state);
        }
    }

    public void setState(ChassisSpeeds goalSpeeds, Rotation2d goalAngle) {
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(goalSpeeds, goalAngle);
        state.speedMetersPerSecond = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        state.angle = goalAngle; 
    }

    public double getMaxLinearSpeed() {
        return maxLinearSpeed;
    }

    public double getMaxAngularSpeed() {
        return maxAngularSpeed;
    }
    
}
