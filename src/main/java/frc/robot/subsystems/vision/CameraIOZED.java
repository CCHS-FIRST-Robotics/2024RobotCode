package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.AprilTag;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class CameraIOZED implements CameraIO {

    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("tags");

    
    /**
     * Constructs a new CameraIOZED object
     */
    public CameraIOZED() {
        System.out.println("[Init] Creating CameraIOZED");
    }

    /* (non-Javadoc)
     * @see frc.robot.subsystems.vision.CameraIO#updateInputs(frc.robot.subsystems.vision.CameraIO.CameraIOInputs)
     */
    public void updateInputs(CameraIOInputs inputs) {
        // Pose estimate from the zed (x, y, theta)
        double[] pose2d = tagsTable.getEntry("pose_estimate").getDoubleArray(new double[] {-1, -1, -1});
        double[] pose3d = tagsTable.getEntry("pose_estimate_3d").getDoubleArray(new double[] {-1, -1, -1, -1, -1, -1});
        inputs.poseEstimate = new Pose2d(pose2d[0], pose2d[1], new Rotation2d(pose2d[2]));
        inputs.poseEstimate3d = new Pose3d(pose3d[0], pose3d[1], pose3d[2], new Rotation3d(pose2d[3], pose2d[4], pose2d[5]));

        // Values from the primary (closest) tag
        inputs.primaryTagId = (int) tagsTable.getEntry("primary_tag_id").getDouble(-1);
        inputs.primaryTagX = tagsTable.getEntry("primary_tag_x").getDouble(-1);
        inputs.primaryTagY = tagsTable.getEntry("primary_tag_y").getDouble(-1);
        inputs.primaryTagZ = tagsTable.getEntry("primary_tag_z").getDouble(-1);
        inputs.primaryTagHeading = tagsTable.getEntry("primary_tag_heading").getDouble(-1);

        // Values for all tags found by the camera
        double[] tagIds = tagsTable.getEntry("tag_ids").getDoubleArray(new double[] {});
        double[] tagXs = tagsTable.getEntry("tag_xs").getDoubleArray(new double[] {});
        // inputs.tagYs = tagsTable.getEntry("tag_ys").getDoubleArray(new double[] {});
        double[] tagZs = tagsTable.getEntry("tag_zs").getDoubleArray(new double[] {});
        double[] tagHeadings = tagsTable.getEntry("tag_headings").getDoubleArray(new double[] {});

        ArrayList<AprilTag> tags = new ArrayList<AprilTag>(tagIds.length);
        for (int i = 0; i < tagIds.length; i++) {
            tags.add(new AprilTag((int) tagIds[i], tagXs[i], tagZs[i], tagHeadings[i]));
        }
        inputs.tags = tags;
    }
}
