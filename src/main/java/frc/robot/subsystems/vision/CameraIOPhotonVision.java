package frc.robot.subsystems.vision;

import org.photonvision.*;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.*;
import java.util.*;

public class CameraIOPhotonVision implements CameraIO {
    PhotonCamera camera = new PhotonCamera("limelight");
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
            AprilTagFields.kDefaultField.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            new Transform3d() // ! most likely wrong bc camera isn't at the middle of the robot
    );

    EstimatedRobotPose robotPose = new EstimatedRobotPose(null, 0, null, null);

    public CameraIOPhotonVision() {
        System.out.println("[Init] Creating CameraIOPhotonVision");

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        Optional<EstimatedRobotPose> visionEstimate = poseEstimator.update();
        if (visionEstimate.isPresent()) {
            robotPose = visionEstimate.get();
        }
    }
}