package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.utils.AprilTag;

import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

public interface CameraIO {
    public static class CameraIOInputs implements LoggableInputs {
        // Values from the primary (closest) tag
        public int primaryTagId = -1;
        public Measure<Distance> primaryTagX = Meters.of(-1);
        public Measure<Distance> primaryTagY = Meters.of(-1);
        public Measure<Distance> primaryTagZ = Meters.of(-1);
        public Measure<Angle> primaryTagHeading = Radians.of(-1);

        // Values for all tags found by the camera
        int numTags = 0;
        public ArrayList<AprilTag> tags = new ArrayList<AprilTag>();

        // Localization data
        Pose2d poseEstimate = new Pose2d(-1, -1, new Rotation2d(-1));
        // double[] poseEstimateArray = new double[] {-1, -1, -1};
        Pose3d poseEstimate3d = new Pose3d(-1, -1, -1, new Rotation3d(-1, -1, -1));

        Measure<Time> timestamp = Seconds.of(0);

        /*
         * IMPLEMENTS LOGGABLE INPUTS MANUALLY (NOT AUTOLOG) TO LOG CUSTOM AprilTag OBJECTS
         */
        @Override
        public void toLog(LogTable table) {
            table.put("primaryTag/Id", primaryTagId);
            table.put("primaryTag/X", primaryTagX);
            table.put("primaryTag/Y", primaryTagY);
            table.put("primaryTag/Z", primaryTagZ);
            table.put("primaryTag/Heading", primaryTagHeading);

            table.put("numTags", tags.size());
            for (int i = 0; i < tags.size(); i++) {
                AprilTag tag = tags.get(i);
                table.put("tag" + i + "/Id", tag.getId());
                table.put("tag" + i + "/Pose", tag.getPose2d());
                table.put("tag" + i + "/Distance", tag.getDistance());
            }

            table.put("poseEstimate2d", poseEstimate);
            table.put("poseEstimate3d", poseEstimate3d);

            table.put("timestampSeconds", timestamp);
        }

        @Override
        public void fromLog(LogTable table) {
            primaryTagId = table.get("primaryTag/Id", primaryTagId);
            primaryTagX = table.get("primaryTag/X", primaryTagX);
            primaryTagY = table.get("primaryTag/Y", primaryTagY);
            primaryTagZ = table.get("primaryTag/Z", primaryTagZ);
            primaryTagHeading = table.get("primaryTag/Heading", primaryTagHeading);

            numTags = table.get("numTags", numTags);
            for (int i = 0; i < numTags; i++) {
                int id = table.get("tag" + i + "/Id", -1);
                Pose2d pose = table.get("tag" + i + "/Pose", new Pose2d(-1, -1, new Rotation2d(-1)));
                tags.add(new AprilTag(id, pose));
            }

            poseEstimate = table.get("poseEstimate2d", poseEstimate);
            poseEstimate3d = table.get("poseEstimate3d", poseEstimate3d);

            timestamp = table.get("timestampSeconds", timestamp);
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(CameraIOInputs inputs) {}
}
