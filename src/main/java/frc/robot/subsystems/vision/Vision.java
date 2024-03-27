package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import java.util.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.vision.CameraIO.CameraIOInputs;
import frc.robot.utils.*;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    CameraIO zed;
    CameraIO photonVision;
    CameraIOInputs ZEDinputs = new CameraIOInputs();
    CameraIOInputs PVinputs = new CameraIOInputs();
    PoseEstimator poseEstimator;
    boolean poseReset = false;

    int i = 0;

    /**
     * Constructs a new Vision object
     * 
     * @param zed          The zed object
     * @param photonVision The PV object
     */
    public Vision(CameraIO zed, CameraIO photonVision) {
        this.zed = zed;
        this.photonVision = photonVision;

        Logger.recordOutput("AprilTagLocations", Constants.APRIL_TAG_LOCATIONS);
    }

    /**
     * (non-Javadoc)
     * 
     * @see edu.wpi.first.wpilibj2.command.Subsystem#periodic()
     */
    public void periodic() {
        zed.updateInputs(ZEDinputs);
        Logger.processInputs("Vision/ZED", ZEDinputs);

        photonVision.updateInputs(PVinputs);
        Logger.processInputs("Vision/PV", PVinputs);

        if (getPoseEstimate3d().pose.getX() > 0) {
            TimestampedPose2d pose = getPoseEstimate();
            if (!poseReset) {
                // poseEstimator.resetPosition(
                // poseEstimator.getPoseEstimate().getRotation(),
                // poseEstimator.getPrevModulePositions(),
                // pose.pose
                // );
                poseReset = true;
            }
            // System.out.println("Adding Vision Pose");
            // poseEstimator.addVisionData(getPoseEstimate3d(), Timer.getFPGATimestamp())
            // Matrix<N3, N1> visionStdScale = VecBuilder.fill(
            // inputs.primaryTagX,
            // inputs.primaryTagY,
            // Math.hypot(inputs.primaryTagX, inputs.primaryTagY)
            // );
            if (i % 50 == 0)
                System.err.println("pose added");
            Logger.recordOutput("testRecordedPose", pose.pose);
            Logger.recordOutput("testRecordedTimestamp", pose.timestamp);
            // poseEstimator.addVisionMeasurement(pose.pose, pose.timestamp / 1000000,
            // poseEstimator
            // .getDefaultVisionMeasurementStdDevs().times(getTransformToClosestTag().getTranslation().getNorm()));
        }

        if (getZedPoseEstimate().pose.getX() > 0) {
            // TimestampedPose2d pose = getZedPoseEstimate();
            // poseEstimator.addVisionMeasurement(pose.pose, pose.timestamp,
            // getZedPoseStd());
        }

        i++;
        if (i % 20 == 0) {
            System.out.println();
            // System.out.print("Tag Distance: ");
            // System.out.println(inputs.primaryTagX);
            // System.out.print("Pose Estimate Distance: ");
            // System.out.println(inputs.poseEstimate.getX());
        }
    }

    /**
     * Returns the closest tag's transform relative to the robot
     * 
     * @return The closest tag's transform relative to the robot
     */
    public Transform2d getTransformToClosestTag() {
        return new Transform2d(new Translation2d(ZEDinputs.primaryTagX, ZEDinputs.primaryTagY),
                new Rotation2d(ZEDinputs.primaryTagHeading));
    }

    /**
     * Returns the tag with the specified id
     * If there are multiple tags with the same id, the closest one is returned
     * 
     * @param id The id of the tag to return
     * @return The tag with the specified id
     */
    public AprilTag getTagFromId(int id) {
        // Sort tags by distance from robot
        ArrayList<AprilTag> tagsCopy = new ArrayList<AprilTag>(ZEDinputs.tags.size());
        Collections.copy(tagsCopy, ZEDinputs.tags);
        Collections.sort(tagsCopy, (AprilTag a, AprilTag b) -> {
            return Double.compare(
                    a.transform.getTranslation().toTranslation2d().getDistance(new Translation2d()),
                    b.transform.getTranslation().toTranslation2d().getDistance(new Translation2d()));
        });

        // Return the first (closest) tag with the specified id
        for (AprilTag tag : tagsCopy) {
            if (tag.id == id) {
                return tag;
            }
        }
        return null;
    }

    public void setPoseEstimator(PoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    /**
     * Returns the latest pose estimate (2d)
     * 
     * @return The latest pose estimate (2d)
     */
    public TimestampedPose2d getPoseEstimate() {
        return ZEDinputs.tagBasedPoseEstimate;
    }

    /**
     * Returns the latest pose estimate (3d)
     * 
     * @return The latest pose estimate (3d)
     */
    public TimestampedPose3d getPoseEstimate3d() {
        return ZEDinputs.tagBasedPoseEstimate3d;
    }

    /**
     * Returns the latest pose estimate (2d)
     * 
     * @return The latest pose estimate (2d)
     */
    public TimestampedPose2d getZedPoseEstimate() {
        return ZEDinputs.zedPoseEstimate;
    }

    /**
     * Returns the latest pose estimate (3d)
     * 
     * @return The latest pose estimate (3d)
     */
    public TimestampedPose3d getZedPoseEstimate3d() {
        return ZEDinputs.zedBasedPoseEstimate3d;
    }

    public Matrix<N3, N1> getZedPoseStd() {
        return VecBuilder.fill(
                Math.sqrt(ZEDinputs.zedBasedPoseCovar.get(0, 0)),
                Math.sqrt(ZEDinputs.zedBasedPoseCovar.get(1, 0)),
                Math.sqrt(ZEDinputs.zedBasedPoseCovar.get(2, 0)));
    }
}
