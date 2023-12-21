package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import frc.robot.utils.AprilTag;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class CameraIOZED implements CameraIO {

    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("tags");

    DoubleArraySubscriber pose2dSub = tagsTable.getDoubleArrayTopic("pose_estimate").subscribe(new double[] {-1, -1, -1});
    DoubleArraySubscriber pose3dSub = tagsTable.getDoubleArrayTopic("pose_estimate_3d").subscribe(new double[] {-1, -1, -1, -1, -1, -1});

    DoubleSubscriber primaryTagIdSub = tagsTable.getDoubleTopic("primary_tag_id").subscribe(-1);
    DoubleSubscriber primaryTagXSub = tagsTable.getDoubleTopic("primary_tag_x").subscribe(-1);
    DoubleSubscriber primaryTagYSub = tagsTable.getDoubleTopic("primary_tag_y").subscribe(-1);
    DoubleSubscriber primaryTagZSub = tagsTable.getDoubleTopic("primary_tag_z").subscribe(-1);
    DoubleSubscriber primaryTagHeadingSub = tagsTable.getDoubleTopic("primary_tag_heading").subscribe(-1);

    DoubleArraySubscriber tagIdsSub = tagsTable.getDoubleArrayTopic("tag_ids").subscribe(new double[] {});
    DoubleArraySubscriber tagXsSub = tagsTable.getDoubleArrayTopic("tag_xs").subscribe(new double[] {});
    DoubleArraySubscriber tagYsSub = tagsTable.getDoubleArrayTopic("tag_ys").subscribe(new double[] {});
    DoubleArraySubscriber tagZsSub = tagsTable.getDoubleArrayTopic("tag_zs").subscribe(new double[] {});
    DoubleArraySubscriber tagHeadingsSub = tagsTable.getDoubleArrayTopic("tag_headings").subscribe(new double[] {});

    
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
        double[] pose2d = pose2dSub.get();
        double[] pose3d = pose3dSub.get();

        inputs.poseEstimate = new Pose2d(pose2d[0], pose2d[1], new Rotation2d(Units.degreesToRadians(pose2d[2])));
        inputs.poseEstimateArray = pose2d;
        inputs.poseEstimate3d = new Pose3d(pose3d[0], pose3d[1], pose3d[2], new Rotation3d(pose3d[3], pose3d[4], pose3d[5]));

        // Values from the primary (closest) tag
        inputs.primaryTagId = (int) primaryTagIdSub.get();
        inputs.primaryTagX = primaryTagXSub.get();
        inputs.primaryTagY = primaryTagYSub.get();
        // inputs.primaryTagZ = primaryTagZSub.get();
        inputs.primaryTagHeading = primaryTagHeadingSub.get();

        // Values for all tags found by the camera
        double[] tagIds = tagIdsSub.get();
        double[] tagXs = tagXsSub.get();
        double[] tagYs = tagYsSub.get();
        // double[] tagZs = tagZsSub.get();
        double[] tagHeadings = tagHeadingsSub.get();

        ArrayList<AprilTag> tags = new ArrayList<AprilTag>(tagIds.length);
        for (int i = 0; i < tagIds.length; i++) {
            tags.add(
                new AprilTag(
                    (int) tagIds[i], 
                    tagXs[i], 
                    tagYs[i], 
                    tagHeadings[i]
                )
            );
        }
        inputs.tags = tags;

        // Convert from ns to sec
        inputs.timestampSeconds = pose2dSub.getLastChange() / 1000000;
    }
}
