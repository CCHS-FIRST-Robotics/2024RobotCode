package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.networktables.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import frc.robot.utils.*;
import frc.robot.utils.AprilTag;

public class CameraIOZED implements CameraIO {
    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("tags");

    // DoubleArraySubscriber tagPose2dSub =
    // tagsTable.getDoubleArrayTopic("pose_estimate").subscribe(new double[] {-1,
    // -1, -1});
    DoubleArraySubscriber tagPose3dSub = tagsTable.getDoubleArrayTopic("pose_estimate_3d")
            .subscribe(new double[] { -1, -1, -1, -1, -1, -1 });
    DoubleArraySubscriber zedPose3dSub = tagsTable.getDoubleArrayTopic("zed_pose_estimate")
            .subscribe(new double[] { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 });

    DoubleSubscriber primaryTagIdSub = tagsTable.getDoubleTopic("primary_tag_id").subscribe(-1);
    DoubleSubscriber primaryTagXSub = tagsTable.getDoubleTopic("primary_tag_x").subscribe(-1);
    DoubleSubscriber primaryTagYSub = tagsTable.getDoubleTopic("primary_tag_y").subscribe(-1);
    DoubleSubscriber primaryTagZSub = tagsTable.getDoubleTopic("primary_tag_z").subscribe(-1);
    DoubleSubscriber primaryTagRollSub = tagsTable.getDoubleTopic("primary_tag_roll").subscribe(-1);
    DoubleSubscriber primaryTagPitchSub = tagsTable.getDoubleTopic("primary_tag_pitch").subscribe(-1);
    DoubleSubscriber primaryTagHeadingSub = tagsTable.getDoubleTopic("primary_tag_heading").subscribe(-1);

    DoubleArraySubscriber tagIdsSub = tagsTable.getDoubleArrayTopic("tag_ids").subscribe(new double[] {});
    DoubleArraySubscriber tagXsSub = tagsTable.getDoubleArrayTopic("tag_xs").subscribe(new double[] {});
    DoubleArraySubscriber tagYsSub = tagsTable.getDoubleArrayTopic("tag_ys").subscribe(new double[] {});
    DoubleArraySubscriber tagZsSub = tagsTable.getDoubleArrayTopic("tag_zs").subscribe(new double[] {});
    DoubleArraySubscriber tagRollsSub = tagsTable.getDoubleArrayTopic("tag_rolls").subscribe(new double[] {});
    DoubleArraySubscriber tagPitchesSub = tagsTable.getDoubleArrayTopic("tag_pitches").subscribe(new double[] {});
    DoubleArraySubscriber tagHeadingsSub = tagsTable.getDoubleArrayTopic("tag_headings").subscribe(new double[] {});

    /**
     * Constructs a new CameraIOZED object
     */
    public CameraIOZED() {
        System.out.println("[Init] Creating CameraIOZED");
    }

    /*
     * (non-Javadoc)
     * 
     * @see
     * frc.robot.subsystems.vision.CameraIO#updateInputs(frc.robot.subsystems.vision
     * .CameraIO.CameraIOInputs)
     */
    public void updateInputs(CameraIOInputs inputs) {
        // Pose estimate from the zed (x, y, theta)
        // double[] pose2d = tagPose2dSub.get();
        TimestampedDoubleArray pose3d = tagPose3dSub.getAtomic();

        // inputs.poseEstimate = new Pose2d(pose2d[0], pose2d[1], new
        // Rotation2d(Degrees.of(pose2d[2]).in(Radians)));
        // inputs.poseEstimateArray = pose2d;

        Pose3d pose = new Pose3d(
                pose3d.value[0], pose3d.value[1], pose3d.value[2],
                new Rotation3d(pose3d.value[3], pose3d.value[4], pose3d.value[5]));
        inputs.tagBasedPoseEstimate3d = new TimestampedPose3d(pose, pose3d.timestamp);
        inputs.tagBasedPoseEstimate = new TimestampedPose2d(pose.toPose2d(), pose3d.timestamp);

        TimestampedDoubleArray zedPose = zedPose3dSub.getAtomic(); // (0-5 pose, 6-11 covar)
        pose = new Pose3d(
                zedPose.value[0], zedPose.value[1], zedPose.value[2],
                new Rotation3d(zedPose.value[3], zedPose.value[4], zedPose.value[5]));
        inputs.zedBasedPoseEstimate3d = new TimestampedPose3d(pose, zedPose.timestamp);
        inputs.zedPoseEstimate = new TimestampedPose2d(pose.toPose2d(), zedPose.timestamp);
        inputs.zedBasedPoseCovar = VecBuilder.fill(zedPose.value[6], zedPose.value[7], zedPose.value[11]); // (x,
                                                                                                           // y,
                                                                                                           // yaw)
        // Values from the primary (closest) tag
        inputs.primaryTagId = (int) primaryTagIdSub.get();
        inputs.primaryTagX = Meters.of(primaryTagXSub.get());
        inputs.primaryTagY = Meters.of(primaryTagYSub.get());
        inputs.primaryTagZ = Meters.of(primaryTagZSub.get());
        inputs.primaryTagRoll = Radians.of(primaryTagRollSub.get());
        inputs.primaryTagPitch = Radians.of(primaryTagPitchSub.get());
        inputs.primaryTagHeading = Radians.of(primaryTagHeadingSub.get());

        // Values for all tags found by the camera
        double[] tagIds = tagIdsSub.get();
        double[] tagXs = tagXsSub.get();
        double[] tagYs = tagYsSub.get();
        double[] tagZs = tagZsSub.get();
        double[] tagRolls = tagRollsSub.get();
        double[] tagPitches = tagPitchesSub.get();
        double[] tagHeadings = tagHeadingsSub.get();

        if (tagIds.length != tagXs.length || tagIds.length != tagYs.length || tagIds.length != tagZs.length
                || tagIds.length != tagRolls.length || tagIds.length != tagPitches.length
                || tagIds.length != tagHeadings.length) {
            System.out.println("Timestamp desync, skipping tag array update");
            return;
        }

        inputs.tags.clear();
        for (int i = 0; i < tagIds.length; i++) {
            inputs.tags.add(
                    new AprilTag(
                            (int) tagIds[i],
                            new Pose3d(tagXs[i], tagYs[i], tagZs[i],
                                    new Rotation3d(tagRolls[i], tagPitches[i],
                                            tagHeadings[i]))));
        }
    }
}