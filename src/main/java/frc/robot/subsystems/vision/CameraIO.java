package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.utils.AprilTag;

public interface CameraIO {

    @AutoLog
    public static class CameraIOInputs {
        // Values from the primary (closest) tag
        public int primaryTagId = -1;
        public double primaryTagX = -1;
        public double primaryTagY = -1;
        public double primaryTagZ = -1;
        public double primaryTagHeading = -1;

        // Values for all tags found by the camera
        public ArrayList<AprilTag> tags = new ArrayList<AprilTag>();

        // Localization data
        Pose2d poseEstimate = new Pose2d();
        Pose3d poseEstimate3d = new Pose3d();

        double timestampSeconds = 0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(CameraIOInputs inputs) {}
}
