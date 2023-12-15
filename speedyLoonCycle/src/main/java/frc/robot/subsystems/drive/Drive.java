package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{
    private Module one, two, three, four;
    private ChassisSpeeds robotSpeeds;

    public Drive(){
        robotSpeeds = new ChassisSpeeds();
        one = new Module();
        two = new Module();
        three = new Module();
        four = new Module();
    }

    @Override
    public void periodic(){

    }

}
