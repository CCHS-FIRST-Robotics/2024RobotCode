package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.vision.CameraIO.CameraIOInputs;
import frc.robot.utils.*;

public class Vision extends SubsystemBase {
    CameraIO zed;
    CameraIO photonVision;
    CameraIOInputs ZEDinputs = new CameraIOInputs();
    CameraIOInputs PVinputs = new CameraIOInputs();
    PoseEstimator poseEstimator;
    boolean poseReset = false;

    public Vision(CameraIO zed, CameraIO photonVision) {
        this.zed = zed;
        this.photonVision = photonVision;
    }

    public void setPoseEstimator(PoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    public void periodic() {
        zed.updateInputs(ZEDinputs);
        Logger.processInputs("Vision/ZED", ZEDinputs);

        photonVision.updateInputs(PVinputs);
        Logger.processInputs("Vision/PV", PVinputs);

        // add the PV pose estimate to poseEstimator
        if (PVinputs.tagBasedPoseEstimate.pose.getX() > 0 && PVinputs.primaryTagAmbiguity < .2) {
            TimestampedPose2d pose = PVinputs.tagBasedPoseEstimate;

            Logger.recordOutput("testRecordedPosePV", pose.pose);
            Logger.recordOutput("testRecordedTimestampPV", pose.timestamp);
            poseEstimator.addVisionMeasurement(
                    pose.pose,
                    pose.timestamp,
                    poseEstimator.getDefaultPVMeasurementStdDevs()
                            .times(new Translation2d(PVinputs.primaryTagX, PVinputs.primaryTagY).getNorm()));
        }
    }
}