package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class CameraIOZED implements CameraIO {

    NetworkTable tags = NetworkTableInstance.getDefault().getTable("tags");

    
    public CameraIOZED() {
        System.out.println("[Init] Creating CameraIOZED");
    }

    public void updateInputs(CameraIOInputs inputs) {
        // Pose estimate from the zed (x, y, theta)
        double[] pose2d = tags.getEntry("zed_pose").getDoubleArray(new double[] {-1, -1, -1});
        // inputs.poseEstimate = new Pose2d(pose2d[0], pose2d[1], new Rotation2d(pose2d[2]));

        // Values from the primary (closest) tag
        inputs.primaryTagId = (int) tags.getEntry("primary_tag_id").getDouble(-1);
        inputs.primaryTagX = tags.getEntry("primary_tag_x").getDouble(-1);
        inputs.primaryTagY = tags.getEntry("primary_tag_y").getDouble(-1);
        inputs.primaryTagZ = tags.getEntry("primary_tag_z").getDouble(-1);
        inputs.primaryTagHeading = tags.getEntry("primary_tag_heading").getDouble(-1);

        // Values for all tags found by the camera
        double[] tagIds = tags.getEntry("tag_ids").getDoubleArray(new double[] {});
        inputs.tagIds = new long[tagIds.length];
        for (int i = 0; i < tagIds.length; i++) {
            inputs.tagIds[i] = (int) tagIds[i];
        }
        inputs.tagXs = tags.getEntry("tag_xs").getDoubleArray(new double[] {});
        inputs.tagYs = tags.getEntry("tag_ys").getDoubleArray(new double[] {});
        inputs.tagZs = tags.getEntry("tag_zs").getDoubleArray(new double[] {});
        inputs.tagHeadings = tags.getEntry("tag_headings").getDoubleArray(new double[] {});
    }
}
