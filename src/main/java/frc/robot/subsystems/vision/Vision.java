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

    public Vision(CameraIO io, PoseEstimator poseEstimator) {
        this.io = io;
        this.poseEstimator = poseEstimator;
    }
    
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Vision", inputs);

        if (getPoseEstimate3d().getX() > 0) {
            // poseEstimator.addVisionData(getPoseEstimate3d(), Timer.getFPGATimestamp());
        } 
    }

    public Transform2d getTransformToClosestTag() {
        return new Transform2d(new Translation2d(inputs.primaryTagZ, inputs.primaryTagX), new Rotation2d(inputs.primaryTagHeading));
    }

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

    public Pose2d getPoseEstimate() {
        return inputs.poseEstimate;
    }

    public Pose3d getPoseEstimate3d() {
        return inputs.poseEstimate3d;
    }
}
