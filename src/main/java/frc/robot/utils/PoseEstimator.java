package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class PoseEstimator extends SwerveDrivePoseEstimator {

    double latestTimestamp = Timer.getFPGATimestamp();
    Pose2d poseEstimate = new Pose2d();
    Pose3d poseEstimate3d = new Pose3d();

    // LinearFilter visionXFilter = LinearFilter.movingAverage(10);
    LinearFilter visionYFilter = LinearFilter.movingAverage(30);

    MedianFilter visionXFilter = new MedianFilter(10);
    // MedianFilter visionYFilter = new MedianFilter(20);
    // KalmanFilter
    // LinearQuadraticRegulator

    // static final Matrix<N3, N1> defaultStateStdDevs = VecBuilder.fill(10, 10,
    // 10); // for testing (only use vision, essentially)
    static final Matrix<N3, N1> defaultStateStdDevs = VecBuilder.fill(0.003, 0.003, 0.0002);

    SwerveModulePosition[] prevModulePositions = new SwerveModulePosition[4];

    /*
     * .002, .035, .035 at .5m
     * .025, .145, .17 at 1m
     * .022, .085, .08 at 1.5m
     * .01, .045, .1 at 2m
     * .007, .188, .1 at 2.5m
     * .045, .16, .1 at 3m
     * Linear fit with distance -> slope: 0.01, 0.05, .1
     */
    static final Matrix<N3, N1> defaultPVMeasurementStdDevs = VecBuilder.fill(.06, .08, 2);

    /**
     * Constructs a new PoseEstimator object
     */
    public PoseEstimator(SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters) {
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters, defaultStateStdDevs,
                defaultPVMeasurementStdDevs);
    }
    public Pose2d getPoseEstimate() {
        return getEstimatedPosition();
    }

    public Matrix<N3, N1> getDefaultPVMeasurementStdDevs() {
        return defaultPVMeasurementStdDevs;
    }

    @Override
    public Pose2d updateWithTime(double timestamp, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        // System.out.println("Adding odom measurement");
        Logger.recordOutput("odomTimestamp", timestamp);
        prevModulePositions = modulePositions;
        return super.updateWithTime(timestamp, gyroAngle, modulePositions);
    }
}
