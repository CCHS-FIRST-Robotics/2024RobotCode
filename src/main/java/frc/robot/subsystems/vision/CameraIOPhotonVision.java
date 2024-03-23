package frc.robot.subsystems.vision;

import org.photonvision.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.*;
import java.util.*;

import frc.robot.utils.AprilTag;

public class CameraIOPhotonVision implements CameraIO {
    PhotonCamera camera = new PhotonCamera("limelight");
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
            AprilTagFields.kDefaultField.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            new Transform3d() // ! most likely wrong bc camera isn't at the middle of the robot
    );
    private double lastEstTimestamp = 0;

    public CameraIOPhotonVision() {
        System.out.println("[Init] Creating CameraIOPhotonVision");
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // returns latest estimated robot pose
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = poseEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) {
            lastEstTimestamp = latestTimestamp;
        }
        return visionEst;
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        var visionEst = getEstimatedGlobalPose();
        visionEst.ifPresent(est -> {
            var estPose = est.estimatedPose;
            var result = camera.getLatestResult();
            // result.getTargets();
            PhotonTrackedTarget closest = null;
            double dist = 0.0;
            for (PhotonTrackedTarget smart : result.getTargets()) {
                inputs.tags.add(new AprilTag(smart.getFiducialId(), smart.getBestCameraToTarget()));
                if (closest == null || dist > calcMag(smart.getBestCameraToTarget())) {
                    closest = smart;
                    dist = calcMag(smart.getBestCameraToTarget());
                }
            }
            inputs.numTags = inputs.tags.size();
            if (closest != null) {
                inputs.primaryTagId = closest.getFiducialId();
                inputs.primaryTagX = Meters.of(closest.getBestCameraToTarget().getX());
                inputs.primaryTagY = Meters.of(closest.getBestCameraToTarget().getY());
                inputs.primaryTagZ = Meters.of(closest.getBestCameraToTarget().getZ());
                inputs.primaryTagPitch = Radians.of(closest.getPitch());

                closest.getBestCameraToTarget().getX();
            }
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

        });
    }

    public static double calcMag(Transform3d vector) {
        return Math.sqrt(vector.getX() * vector.getX() + vector.getY() * vector.getY() + vector.getZ() * vector.getZ());
    }

    // fill tags arraylist
    // inputs.tags.add

    // var visionEst = getEstimatedGlobalPose();
    // visionEst.ifPresent(
    // est -> {
    // var estPose = est.estimatedPose.toPose2d();
    // // Change our trust in the measurement based on the tags we can see
    // Matrix<N3, N1> estStdDevs = getEstimatedStandardDeviations(estPose);

    // drivetrain.addVisionMeasurement(
    // est.estimatedPose.toPose2d(), , estStdDevs);
    // });

    // TimestampedPose2d pose = new TimestampedPose2d(est.estimatedPose.toPose2d(),
    // est.timestampSeconds)
    // }
}