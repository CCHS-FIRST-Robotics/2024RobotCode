package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.*;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.targeting.*;
import edu.wpi.first.math.geometry.*;
import java.util.*;
import frc.robot.utils.*;

public class CameraIOPhotonVision implements CameraIO {
    PhotonCamera camera = new PhotonCamera("limelight");
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
            AprilTagFields.kDefaultField.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            new Transform3d() // ! most likely wrong bc camera isn't at the middle of the robot
    );

    public CameraIOPhotonVision() {
        System.out.println("[Init] Creating CameraIOPhotonVision");

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        Optional<EstimatedRobotPose> estimate = poseEstimator.update();

        if (estimate.isPresent()) {
            PhotonPipelineResult cameraResult = camera.getLatestResult();
            Pose3d estimatedPose3d = estimate.get().estimatedPose;
            double time = cameraResult.getTimestampSeconds();
            inputs.tagPoseEstimate = new TimestampedPose2d(estimatedPose3d.toPose2d(), time);
            inputs.tagPoseEstimate3d = new TimestampedPose3d(estimatedPose3d, time);

            // get closest tags
            PhotonTrackedTarget closestTag = null;
            double dist = 0.0;
            for (PhotonTrackedTarget smart : cameraResult.getTargets()) {
                inputs.tags.add(new AprilTag(smart.getFiducialId(), smart.getBestCameraToTarget()));
                if (closestTag == null || dist > calcMag(smart.getBestCameraToTarget())) {
                    closestTag = smart;
                    dist = calcMag(smart.getBestCameraToTarget());
                }
            }

            // add to inputs
            if (closestTag != null) {
                inputs.primaryTagId = closestTag.getFiducialId();
                inputs.primaryTagX = Meters.of(closestTag.getBestCameraToTarget().getX());
                inputs.primaryTagY = Meters.of(closestTag.getBestCameraToTarget().getY());
                inputs.primaryTagZ = Meters.of(closestTag.getBestCameraToTarget().getZ());
                inputs.primaryTagPitch = Radians.of(closestTag.getPitch());

                closestTag.getBestCameraToTarget().getX();
            }
            inputs.numTags = inputs.tags.size();

            // List<edu.wpi.first.apriltag.AprilTag> tags =
            // poseEstimator.getFieldTags().getTags();
            // frc.robot.utils.AprilTag closeet = null;
            // for(AprilTag at : tags){
            // // inputs.tags.add(new frc.robot.utils.AprilTag(at.ID, at.pose));
            // Transform3d dist = estPose.minus(at.pose);
            // inputs.tags.add(new frc.robot.utils.AprilTag(at.ID, dist));
            // // if(closeset == null || dist.)

            // // at.pose));
            // }
            // inputs.numTags = inputs.tags.size();
        }
    }

    public static double calcMag(Transform3d vector) {
        return Math.sqrt(vector.getX() * vector.getX() + vector.getY() * vector.getY() + vector.getZ() * vector.getZ());
    }
}