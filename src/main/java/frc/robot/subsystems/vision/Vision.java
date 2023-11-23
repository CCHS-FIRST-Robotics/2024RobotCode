package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AprilTag;
import frc.robot.utils.PoseEstimator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    
    CameraIO io;
    CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();
    PoseEstimator poseEstimator;

    /**
     * Constructs a new Vision object
     * 
     * @param io The camera IO object
     * @param poseEstimator The pose estimator
     */
    public Vision(CameraIO io, PoseEstimator poseEstimator) {
        this.io = io;
        this.poseEstimator = poseEstimator;
    }
    
    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj2.command.Subsystem#periodic()
     */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Vision", inputs);

        if (getPoseEstimate3d().getX() > 0) {
            // poseEstimator.addVisionData(getPoseEstimate3d(), Timer.getFPGATimestamp());
        } 
    }

    /**
     * Returns the closest tag's transform relative to the robot
     * 
     * @return The closest tag's transform relative to the robot
     */
    public Transform2d getTransformToClosestTag() {
        return new Transform2d(new Translation2d(inputs.primaryTagZ, inputs.primaryTagX), new Rotation2d(inputs.primaryTagHeading));
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
                a.transform.getTranslation().getDistance(new Translation2d()),
                b.transform.getTranslation().getDistance(new Translation2d())
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

    /**
     * Returns the latest pose estimate (2d)
     * 
     * @return The latest pose estimate (2d)
     */
    public Pose2d getPoseEstimate() {
        return inputs.poseEstimate;
    }

    /**
     * Returns the latest pose estimate (3d)
     * 
     * @return The latest pose estimate (3d)
     */
    public Pose3d getPoseEstimate3d() {
        return inputs.poseEstimate3d;
    }
}
