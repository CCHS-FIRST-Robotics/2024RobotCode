package frc.robot.subsystems.mecaDrive;
import frc.robot.Constants;
import frc.robot.Constants.HardwareConstants;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mecaDrive.DriveIOInputsAutoLogged;
import frc.robot.subsystems.swerveDrive.GyroIO;
import frc.robot.subsystems.swerveDrive.GyroIO.GyroIOInputs;
import frc.robot.subsystems.swerveDrive.GyroIOInputsAutoLogged;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;

public class Drive extends SubsystemBase {
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private final Measure<Velocity<Distance>> maxLinearVelocity = MetersPerSecond.of(3); // meters per second
  private final Measure<Velocity<Velocity<Distance>>> maxLinearAcceleration = MetersPerSecondPerSecond.of(3); // meters per second squared

  private final Measure<Velocity<Angle>> maxAngularVelocity = RadiansPerSecond.of(2 * Math.PI); // radians per second
  private final Measure<Velocity<Velocity<Angle>>> maxAngularAcceleration = RadiansPerSecond.per(Second).of(2 * Math.PI); // radians per second squared

  // Odometry class for tracking robot pose
  MecanumDriveOdometry odometry =
  new MecanumDriveOdometry(
        HardwareConstants.MECANUM_KINEMATICS,
        new Rotation2d(),
        new MecanumDriveWheelPositions()
  );

  double thetaKp = 0.0;
  double thetaKi = 0.0;
  double thetaKd = 0.0;
  PIDController thetaController = new PIDController(thetaKp, thetaKi, thetaKd);

  /** Creates a new Drive. */
  public Drive(DriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    // Update odometry and log the new pose
    odometry.update(getYaw(), getWheelPositions());
    Logger.recordOutput("Odometry", getPose());
  }

  /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
     * speeds have no effect on the angular speed.
     *
     * @param xSpeed Speed of the robot in the x direction (forward/backwards).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
      WheelSpeeds speeds;
      if (fieldRelative) {
        speeds = MecanumDrive.driveCartesianIK(xSpeed, ySpeed, rot, getYaw());
      } else {
        speeds = MecanumDrive.driveCartesianIK(xSpeed, ySpeed, rot);
      }
      io.setVoltage(speeds.frontRight * 12, speeds.frontLeft * 12, speeds.rearRight * 12, speeds.rearLeft * 12);
  }

  /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
     * speeds have no effect on the angular speed.
     *
     * @param xSpeed Speed of the robot in the x direction (forward/backwards).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
      
      if (fieldRelative) {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          chassisSpeeds.vxMetersPerSecond,
          chassisSpeeds.vyMetersPerSecond,
          chassisSpeeds.omegaRadiansPerSecond,
          getYaw()
        );
      }
      chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, Constants.PERIOD);
      MecanumDriveWheelSpeeds speeds = getKinematics().toWheelSpeeds(chassisSpeeds);
      io.setVelocity(
        speeds.frontRightMetersPerSecond, 
        speeds.frontLeftMetersPerSecond, 
        speeds.rearRightMetersPerSecond, 
        speeds.rearLeftMetersPerSecond
      );
  }

  /** Stops the drive. */
  public void stop() {
    io.setVoltage(0, 0, 0 ,0);
  }

  /** Returns the current odometry pose in meters. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getWheelPositions(), pose);
  }

  public Rotation2d getYaw() {
    return new Rotation2d(-inputs.gyroYawRad);
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
      convertPosition(inputs.flPositionRaw),
      convertPosition(inputs.frPositionRaw),
      convertPosition(inputs.blPositionRaw),
      convertPosition(inputs.brPositionRaw)
    );
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      convertVelocity(inputs.flVelocityRaw),
      convertVelocity(inputs.frVelocityRaw),
      convertVelocity(inputs.blVelocityRaw),
      convertVelocity(inputs.brVelocityRaw)
    );
  }

  public Twist2d getVelocity() {
    ChassisSpeeds speeds = getKinematics().toChassisSpeeds(getWheelSpeeds());
    return new Twist2d(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond, 
      inputs.gyroConnected ? inputs.gyroYawVelocity : speeds.omegaRadiansPerSecond
    );
  }

  /**
	 * Converts raw position units to meters
	 * @param rawPosition the position from an encoder in raw sensor units
	 * @return the position in meters
	 */
	private double convertPosition(double rawPosition) {
		// raw units are "clicks," so divide by "clicks" per rotation to get rotations
		// also account for gear ratio because the encoders measure motor output, not actual wheel
		double position = rawPosition / (HardwareConstants.TALON_FX_CPR * HardwareConstants.FALCON_GEARBOX_RATIO);
		// multiply by circumference to get linear distance
		position *= Math.PI * HardwareConstants.MECANUM_WHEEL_DIAMETER;
		return position;
	}

   /**
	 * Converts raw sensor velocity to meters/second
	 * @param rawVelocity the velocity from an encoder in raw sensor units
	 * @return velocity in m/s
	 */
	private double convertVelocity(double rawVelocity) {
		// convert to rotations per second because raw units are "clicks" per 100ms
		// also account for gear ratio because the encoders measure motor output, not actual wheel
		double velocity = rawVelocity / (HardwareConstants.TALON_FX_CPR * HardwareConstants.FALCON_GEARBOX_RATIO) * 10;
		// multiply by circumference to get linear velocity
		velocity *= Math.PI * HardwareConstants.MECANUM_WHEEL_DIAMETER;
		return velocity;
	}

  public MecanumDriveKinematics getKinematics() {
    return HardwareConstants.MECANUM_KINEMATICS;
  }

  public Measure<Velocity<Distance>> getMaxLinearSpeed() {
    return maxLinearVelocity;
  }

  public Measure<Velocity<Velocity<Distance>>> getMaxLinearAcceleration() {
    return maxLinearAcceleration;
  }

  public Measure<Velocity<Angle>> getMaxAngularSpeed() {
    return maxAngularVelocity;
  }

  public Measure<Velocity<Velocity<Angle>>> getMaxAngularAcceleration() {
    return maxAngularAcceleration;
  }

  public PIDController getHeadingController() {
    return thetaController;
  }
}
