package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.subsystems.mecaDrive.Drive;
import java.util.ArrayList;

public class DriveTrajectory {
    
    public ArrayList<Pose2d> positionTrajectory;
    public ArrayList<Twist2d> velocityTrajectory;

    public DriveTrajectory(ArrayList<Pose2d> positionTrajectory, ArrayList<Twist2d> velocityTrajectory) {
        this.positionTrajectory = positionTrajectory;
        this.velocityTrajectory = velocityTrajectory;
    }
}
