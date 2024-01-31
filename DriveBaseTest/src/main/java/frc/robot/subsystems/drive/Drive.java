package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Drive extends SubsystemBase {

    private Module[] modules;
    private ChassisSpeeds speeds;
    private SwerveModuleState state;
    

    public Drive() {
        speeds = new ChassisSpeeds();
        modules = new Module[4];

        for (int i = 0; i < modules.length; i++) {
            
        }
    }

    public void periodic(){
        /*
         * need 4 modules
         * drive for each one
         * 
         * 
         * what is rotation2d
         * 
         * class for modules you wrote
         */

        

    }
    
}
