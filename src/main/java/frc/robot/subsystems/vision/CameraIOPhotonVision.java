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
    PhotonCamera camera = new PhotonCamera("Camera_Module_v1");
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
            AprilTagFields.kDefaultField.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            new Transform3d(
                    new Translation3d(.9, .25, .6),
                    new Rotation3d(0, Radians.convertFrom(20, Degrees), 0)));

    public CameraIOPhotonVision() {
        System.out.println("[Init] Creating CameraIOPhotonVision");

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        // get estimate
        Optional<EstimatedRobotPose> estimate = poseEstimator.update();
        if (!estimate.isPresent()) {
            return;
        }

        // update robot pose
        PhotonPipelineResult cameraResult = camera.getLatestResult();
        Pose3d estimatedPose3d = estimate.get().estimatedPose;
        double time = cameraResult.getTimestampSeconds();
        inputs.tagBasedPoseEstimate = new TimestampedPose2d(estimatedPose3d.toPose2d(), time);
        inputs.tagBasedPoseEstimate3d = new TimestampedPose3d(estimatedPose3d, time);

        // get closest tag
        inputs.tags.clear();
        PhotonTrackedTarget closestTag = null;
        double dist = 0.0;
        for (PhotonTrackedTarget smart : cameraResult.getTargets()) {
            inputs.tags.add(new AprilTag(smart.getFiducialId(), smart.getBestCameraToTarget()));
            double newDist = smart.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
            if (closestTag == null || dist > newDist) {
                closestTag = smart;
                dist = newDist;
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
            inputs.primaryTagRoll = Radians.of(closestTag.getSkew());
            inputs.primaryTagAmbiguity = closestTag.getPoseAmbiguity();
        }
        inputs.numTags = inputs.tags.size();
    }
}