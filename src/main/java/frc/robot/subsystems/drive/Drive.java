package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{
    private Module[] modules;
    private ChassisSpeeds robotSpeeds;
    private Rotation2d robotAngle;
    private SwerveModuleState moduleState;

    public static class Constants{
        public final static double MAGIC_SPEED = 4d;
        public final static double WHEEL_RADIUS_METERS = 0.0508d;
        public final static int SECONDS_IN_A_MINUTE = 60;
        public final static int RADII_IN_A_DIAMETER = 2;
        public final static double DRIVE_GEAR_RATIO = 6.74603174603;
        public final static double TURN_GEAR_RATIO = 21.4285714286;
        
        public final static double getMaxSpeed(){
            return MAGIC_SPEED;
        }

        public final static double getWheelRadiusMeters(){
            return WHEEL_RADIUS_METERS;
        }

        public final static int getSecondsInAMinute(){
            return SECONDS_IN_A_MINUTE;
        }

        public final static int getRadiiInADiameter(){
            return RADII_IN_A_DIAMETER;
        }
        
        public final static double getDriveGearRatio(){
            return DRIVE_GEAR_RATIO;
        }

        public final static double getTurnGearRatio(){
            return TURN_GEAR_RATIO;
        }
        
    }

    public Drive(){
        robotSpeeds = new ChassisSpeeds();
        modules = new Module[4];
        for(int i = 0; i < modules.length; i++){
            modules[i] = new Module(i);
        }
        moduleState = new SwerveModuleState(0, new Rotation2d());
    }

    public void swerveThatShi(SwerveModuleState whatWeWant){
        moduleState = whatWeWant;
    }

    @Override
    public void periodic(){
        // uhh maybe later?
        // robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(goalpos, robotAngle);
        // setState();
        for(int i = 0; i < modules.length; i++){
            modules[i].periodic(); // update only (trust)
            modules[i].driveMotors(moduleState);
        }
    }

    // Foobar (Jk kinda):
    public void setState(ChassisSpeeds relativeFieldPosition, double robotAngle){
        robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(relativeFieldPosition, new Rotation2d(robotAngle));
        moduleState.speedMetersPerSecond = Math.sqrt(
            robotSpeeds.vxMetersPerSecond * robotSpeeds.vxMetersPerSecond + 
            robotSpeeds.vyMetersPerSecond * robotSpeeds.vyMetersPerSecond);
        Rotation2d turning = new Rotation2d(
            Math.asin(robotSpeeds.vxMetersPerSecond / moduleState.speedMetersPerSecond), 
            Math.acos(robotSpeeds.vyMetersPerSecond / moduleState.speedMetersPerSecond));
        moduleState.angle = moduleState.angle.rotateBy(turning);

    }

}
