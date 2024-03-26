package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.*;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.targeting.*;
import edu.wpi.first.math.geometry.*;
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
        // get estimate
        EstimatedRobotPose estimate = poseEstimator.update().isPresent() ? poseEstimator.update().get() : null;

        // update robot pose
        PhotonPipelineResult cameraResult = camera.getLatestResult();
        Pose3d estimatedPose3d = estimate.estimatedPose;
        double time = cameraResult.getTimestampSeconds();
        inputs.tagBasedPoseEstimate = new TimestampedPose2d(estimatedPose3d.toPose2d(), time);
        inputs.tagBasedPoseEstimate3d = new TimestampedPose3d(estimatedPose3d, time);

        // get closest tag
        PhotonTrackedTarget closestTag = null;
        double dist = 0.0;
        for (PhotonTrackedTarget smart : cameraResult.getTargets()) {
            inputs.tags.add(new AprilTag(smart.getFiducialId(), smart.getBestCameraToTarget()));
            if (closestTag == null || dist > calcMag(smart.getBestCameraToTarget())) {
                closestTag = smart;
                dist = calcMag(smart.getBestCameraToTarget());
            }
        }

        // update tag data
        if (closestTag != null) {
            inputs.primaryTagId = closestTag.getFiducialId();
            inputs.primaryTagX = Meters.of(closestTag.getBestCameraToTarget().getX());
            inputs.primaryTagY = Meters.of(closestTag.getBestCameraToTarget().getY());
            inputs.primaryTagZ = Meters.of(closestTag.getBestCameraToTarget().getZ());
            inputs.primaryTagPitch = Radians.of(closestTag.getPitch());
            inputs.primaryTagHeading = Radians.of(closestTag.getYaw());
            inputs.primaryTagPitch = Radians.of(closestTag.getSkew());
            closestTag.getBestCameraToTarget().getX();
        }
        inputs.numTags = inputs.tags.size();
    }

    public static double calcMag(Transform3d vector) {
        return Math.sqrt(vector.getX() * vector.getX() + vector.getY() * vector.getY() + vector.getZ() * vector.getZ());
    }
}