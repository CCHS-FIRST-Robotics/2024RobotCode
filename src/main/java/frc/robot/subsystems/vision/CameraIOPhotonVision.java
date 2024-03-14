// package frc.robot.subsystems.vision;

// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import frc.robot.utils.TimestampedPose2d;

// public class CameraIOPhotonVision implements CameraIO {
    
//     // Change this to match the name of your camera
//     PhotonCamera camera = new PhotonCamera("limelight");
//     PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
//         AprilTagFields.kDefaultField.loadAprilTagLayoutField(),
//         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//         camera,
//         new Transform3d()
//     );

//     private double lastEstTimestamp = 0;

//     /**
//      * Constructs a new CameraIOPhotonVision object
//      */
//     public CameraIOPhotonVision() {
//         System.out.println("[Init] Creating CameraIOPhotonVision");

//         photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//     }

//     @Override
//     public void updateInputs(CameraIOInputs inputs) {
//         var visionEst = getEstimatedGlobalPose();
//         visionEst.ifPresent(
//                 est -> {
//                     var estPose = est.estimatedPose.toPose2d();
//                     // Change our trust in the measurement based on the tags we can see
//                     var estStdDevs = getEstimationStdDevs(estPose);

//                     drivetrain.addVisionMeasurement(
//                             est.estimatedPose.toPose2d(), , estStdDevs);
//                 });

//         TimestampedPose2d pose = new TimestampedPose2d(est.estimatedPose.toPose2d(), est.timestampSeconds)
//     }


//     public PhotonPipelineResult getLatestResult() {
//         return camera.getLatestResult();
//     }


//     /**
//      * The latest estimated robot pose on the field from vision data. This may be empty. This should
//      * only be called once per loop.
//      *
//      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
//      *     used for estimation.
//      */
//     public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
//         var visionEst = photonPoseEstimator.update();
//         double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
//         boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

//         if (newResult) lastEstTimestamp = latestTimestamp;
//         return visionEst;
//     }

//     /**
//      * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
//      * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
//      * This should only be used when there are targets visible.
//      *
//      * @param estimatedPose The estimated pose to guess standard deviations for.
//      */
//     public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
//         var estStdDevs = kSingleTagStdDevs;
//         var targets = getLatestResult().getTargets();
//         int numTags = 0;
//         double avgDist = 0;
//         for (var tgt : targets) {
//             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
//             if (tagPose.isEmpty()) continue;
//             numTags++;
//             avgDist +=
//                     tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
//         }
//         if (numTags == 0) return estStdDevs;
//         avgDist /= numTags;
//         // Decrease std devs if multiple targets are visible
//         if (numTags > 1) estStdDevs = kMultiTagStdDevs;
//         // Increase std devs based on (average) distance
//         if (numTags == 1 && avgDist > 4)
//             estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
//         else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

//         return estStdDevs;
//     }

// }
