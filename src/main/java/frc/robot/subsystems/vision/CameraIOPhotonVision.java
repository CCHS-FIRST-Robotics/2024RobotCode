package frc.robot.subsystems.vision;

import org.photonvision.*;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import java.util.*;
import frc.robot.utils.TimestampedPose2d;

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

    // returns standard deviations of estimated pose
    public Matrix<N3, N1> getEstimatedStandardDeviations(Pose2d estimatedPose) {
        Matrix<N3, N1> estimatedStandardDeviations = kSingleTagStdDevs;
        List<PhotonTrackedTarget> targets = camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) {
            return estimatedStandardDeviations;
        }
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estimatedStandardDeviations = kMultiTagStdDevs;

        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estimatedStandardDeviations = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estimatedStandardDeviations = estimatedStandardDeviations.times(1 + (avgDist * avgDist / 30));
        }

        return estimatedStandardDeviations;
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
    var visionEst = getEstimatedGlobalPose();
    visionEst.ifPresent(
    est -> {
    var estPose = est.estimatedPose.toPose2d();
    // Change our trust in the measurement based on the tags we can see
    Matrix<N3, N1> estStdDevs = getEstimatedStandardDeviations(estPose);

    drivetrain.addVisionMeasurement(
    est.estimatedPose.toPose2d(), , estStdDevs);
    });

    TimestampedPose2d pose = new TimestampedPose2d(est.estimatedPose.toPose2d(),
    est.timestampSeconds)
    }
}