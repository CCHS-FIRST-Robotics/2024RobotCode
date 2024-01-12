package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.CameraIO.CameraIOInputs;
import frc.robot.utils.AprilTag;
import frc.robot.utils.PoseEstimator;
import frc.robot.utils.TimestampedPose2d;
import frc.robot.utils.TimestampedPose3d;
import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    
    CameraIO io;
    CameraIOInputs inputs = new CameraIOInputs();
    PoseEstimator poseEstimator;
    boolean poseReset = false;

    int i = 0;

    /**
     * Constructs a new Vision object
     * 
     * @param io The camera IO object
     * @param poseEstimator The pose estimator
     */
    public Vision(CameraIO io) {
        this.io = io;

        Logger.recordOutput("AprilTagLocations", Constants.APRIL_TAG_LOCATIONS);
    }
    
    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj2.command.Subsystem#periodic()
     */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        if (getPoseEstimate3d().pose.getX() > 0) {
            TimestampedPose2d pose = getPoseEstimate();
            if (!poseReset) {
                poseEstimator.resetPosition(
                    poseEstimator.getPoseEstimate().getRotation(), 
                    poseEstimator.getPrevModulePositions(), 
                    pose.pose
                );
                poseReset = true;
            }
            // System.out.println("Adding Vision Pose");
            // poseEstimator.addVisionData(getPoseEstimate3d(), Timer.getFPGATimestamp())
            // Matrix<N3, N1> visionStdScale = VecBuilder.fill(
            //     inputs.primaryTagX,
            //     inputs.primaryTagY,
            //     Math.hypot(inputs.primaryTagX, inputs.primaryTagY)
            // );
            
            poseEstimator.addVisionMeasurement(pose.pose, pose.timestamp, poseEstimator.getDefaultVisionMeasurementStdDevs().times(getTransformToClosestTag().getTranslation().getNorm()));
        } 
        
        if (getZedPoseEstimate().pose.getX() > 0) {
            // TimestampedPose2d pose = getZedPoseEstimate();
            // poseEstimator.addVisionMeasurement(pose.pose, pose.timestamp, getZedPoseStd());
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
        return new Transform2d(new Translation2d(inputs.primaryTagX, inputs.primaryTagY), new Rotation2d(inputs.primaryTagHeading));
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
        ArrayList<AprilTag> tagsCopy = new ArrayList<AprilTag>(inputs.tags.size());
        Collections.copy(tagsCopy, inputs.tags); 
        Collections.sort(tagsCopy, (AprilTag a, AprilTag b) -> {
            return Double.compare(
                a.transform.getTranslation().toTranslation2d().getDistance(new Translation2d()),
                b.transform.getTranslation().toTranslation2d().getDistance(new Translation2d())
            );
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
        return inputs.tagPoseEstimate;
    }

    /**
     * Returns the latest pose estimate (3d)
     * 
     * @return The latest pose estimate (3d)
     */
    public TimestampedPose3d getPoseEstimate3d() {
        return inputs.tagPoseEstimate3d;
    }

    /**
     * Returns the latest pose estimate (2d)
     * 
     * @return The latest pose estimate (2d)
     */
    public TimestampedPose2d getZedPoseEstimate() {
        return inputs.zedPoseEstimate;
    }

    /**
     * Returns the latest pose estimate (3d)
     * 
     * @return The latest pose estimate (3d)
     */
    public TimestampedPose3d getZedPoseEstimate3d() {
        return inputs.zedPoseEstimate3d;
    }

    public Matrix<N3, N1> getZedPoseStd() {
        return VecBuilder.fill(
            Math.sqrt(inputs.zedPoseCovar.get(0, 0)),
            Math.sqrt(inputs.zedPoseCovar.get(1, 0)),
            Math.sqrt(inputs.zedPoseCovar.get(2, 0))
        );
    }
}
