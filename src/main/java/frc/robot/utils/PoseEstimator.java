package frc.robot.utils;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.*;
import org.littletonrobotics.junction.Logger;

public class PoseEstimator extends SwerveDrivePoseEstimator {
    double latestTimestamp = Timer.getFPGATimestamp();
    Pose2d poseEstimate = new Pose2d();
    Pose3d poseEstimate3d = new Pose3d();

    MedianFilter visionXFilter = new MedianFilter(10); 
    LinearFilter visionYFilter = LinearFilter.movingAverage(30);

    static final Matrix<N3, N1> defaultStateStdDevs = VecBuilder.fill(0.0025, 0.0025, 0.0002);

    SwerveModulePosition[] prevModulePositions = new SwerveModulePosition[4];

    // ! ask colin what this is
    /*
     * .002, .035, .035 at .5m
     * .025, .145, .17 at 1m
     * .022, .085, .08 at 1.5m
     * .01, .045, .1 at 2m
     * .007, .188, .1 at 2.5m
     * .045, .16, .1 at 3m
     * Linear fit with distance -> slope: 0.01, 0.05, .1
     */
    static final Matrix<N3, N1> defaultZEDMeasurementStdDevs = VecBuilder.fill(.025, .15, 1);
    static final Matrix<N3, N1> defaultPVMeasurementStdDevs = VecBuilder.fill(.08, .1, 2);

    public PoseEstimator(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters) 
    {
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters, defaultStateStdDevs, defaultPVMeasurementStdDevs);
    }

    public Pose2d getPoseEstimate() {
        return getEstimatedPosition();
    }

    public Pose2d getPoseEstimateOld() {
        return poseEstimate;
    }

    public Pose3d getPoseEstimate3d() {
        return poseEstimate3d;
    }

    public Matrix<N3, N1> getDefaultStateStdDevs() {
        return defaultStateStdDevs;
    }

    public Matrix<N3, N1> getDefaultZEDMeasurementStdDevs() {
        return defaultZEDMeasurementStdDevs;
    }

    public Matrix<N3, N1> getDefaultPVMeasurementStdDevs() {
        return defaultPVMeasurementStdDevs;
    }

    public SwerveModulePosition[] getPrevModulePositions() {
        return prevModulePositions;
    }

    public void addOdometryData(Twist2d odometryTwist, double timestamp) {
        latestTimestamp = timestamp;

        poseEstimate = poseEstimate.exp(odometryTwist);
        poseEstimate3d = poseEstimate3d.exp(new Twist3d(
                odometryTwist.dx, odometryTwist.dy, 0,
                0, 0, odometryTwist.dtheta));
    }

    public void addVisionData(Pose3d visionPoseEstimate, double timestamp) {
        latestTimestamp = timestamp;

        // This assumes that the vision pose estimate is in the same frame as the gyro
        // Also, kinda assumes that the pose estimate rotation is the same as the gyro rotation
        // (which it should be when there are just these two sources of data and the
        // other is overriding this one)
        Rotation2d gyroAngle = poseEstimate.getRotation();

        // Filter x, y values
        double x = visionXFilter.calculate(visionPoseEstimate.getX());
        double y = visionYFilter.calculate(visionPoseEstimate.getY());

        poseEstimate = new Pose2d(new Translation2d(x, y), gyroAngle);
        poseEstimate3d = new Pose3d(
                visionPoseEstimate.getTranslation(),
                new Rotation3d(
                        visionPoseEstimate.getRotation().getX(),
                        visionPoseEstimate.getRotation().getY(),
                        gyroAngle.getRadians()));
    }

    @Override
    public void addVisionMeasurement(Pose2d poseEstimate, double timestamp) {
        // System.out.println("Adding vision measurement");
        super.addVisionMeasurement(poseEstimate, timestamp);
    }

    @Override
    public void addVisionMeasurement(Pose2d poseEstimate, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        // System.out.println("Adding vision measurement");
        Logger.recordOutput("visionTimestamp", timestamp);
        super.addVisionMeasurement(poseEstimate, timestamp, visionMeasurementStdDevs);
    }

    @Override
    public Pose2d updateWithTime(double timestamp, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        Logger.recordOutput("odomTimestamp", timestamp);
        prevModulePositions = modulePositions;
        return super.updateWithTime(timestamp, gyroAngle, modulePositions);
    }
}