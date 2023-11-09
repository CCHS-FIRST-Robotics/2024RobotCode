package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface CameraIO {

    @AutoLog
    public static class CameraIOInputs {
        // Values from the primary (closest) tag
        public long primaryTagId = -1;
        public double primaryTagX = -1;
        public double primaryTagY = -1;
        public double primaryTagZ = -1;
        public double primaryTagDistance = -1;
        public double primaryTagHeading = -1;

        // Values for all tags found by the camera
        public long[] tagIds = new long[] {};
        public double[] tagXs = new double[] {};
        public double[] tagYs = new double[] {};
        public double[] tagZs = new double[] {};
        public double[] tagDistances = new double[] {};
        public double[] tagHeadings = new double[] {};

        // Localization data
        //TODO: develop localization system
        // Pose2d poseEstimate = new Pose2d();
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(CameraIOInputs inputs) {}
}
