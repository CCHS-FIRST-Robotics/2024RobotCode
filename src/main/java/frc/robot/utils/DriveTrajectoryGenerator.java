package frc.robot.utils;

import edu.wpi.first.math.geometry.*;
import com.choreo.lib.*;
import frc.robot.*;
import java.util.ArrayList;

public final class DriveTrajectoryGenerator {
    public static DriveTrajectory generateChoreoTrajectory(String path) {
        ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(path);

        ArrayList<Pose2d> poseTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> velocityTrajectory = new ArrayList<Twist2d>();

        for (int i = 0; i < (int) (choreoTrajectory.getTotalTime() / Constants.PERIOD) + 2; i++) {
            double time = i * Constants.PERIOD;
            ChoreoTrajectoryState state = choreoTrajectory.sample(time);
            poseTrajectory.add(new Pose2d(state.x, state.y, new Rotation2d(state.heading)));
            velocityTrajectory.add(new Twist2d(state.velocityX, state.velocityY, state.angularVelocity));
        }

        return new DriveTrajectory(poseTrajectory, velocityTrajectory);
    }
}