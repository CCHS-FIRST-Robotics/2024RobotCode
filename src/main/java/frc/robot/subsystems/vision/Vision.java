package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.vision.CameraIO.CameraIOInputs;
import frc.robot.utils.*;

public class Vision extends SubsystemBase {
    CameraIO zed;
    CameraIO photonVision;
    CameraIOInputs ZEDinputs = new CameraIOInputs();
    CameraIOInputs PVinputs = new CameraIOInputs();
    SwerveDrivePoseEstimator poseEstimator;
    boolean poseReset = false;

    static final Matrix<N3, N1> defaultZEDMeasurementStdDevs = VecBuilder.fill(.025, .15, 1);
    static final Matrix<N3, N1> defaultPVMeasurementStdDevs = VecBuilder.fill(.08, .1, 2);

    public Vision(CameraIO zed, CameraIO photonVision) {
        this.zed = zed;
        this.photonVision = photonVision;
    }

    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
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
                    defaultPVMeasurementStdDevs.times(new Translation2d(PVinputs.primaryTagX, PVinputs.primaryTagY).getNorm()));
        }
    }
}