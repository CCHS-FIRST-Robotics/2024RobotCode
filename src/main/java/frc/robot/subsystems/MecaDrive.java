// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;



public class MecaDrive extends SubsystemBase {
 
    // Motor Controller Objects
    private final TalonFX frontLeftMotor = new TalonFX(HardwareConstants.FL_TALON_ID);
    private final TalonFX frontRightMotor = new TalonFX(HardwareConstants.FR_TALON_ID);
    private final TalonFX rearLeftMotor = new TalonFX(HardwareConstants.RL_TALON_ID);
    private final TalonFX rearRightMotor = new TalonFX(HardwareConstants.RR_TALON_ID);

    private final MecanumDrive driveBase = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    

    // Odometry class for tracking robot pose
    MecanumDriveOdometry odometry =
      new MecanumDriveOdometry(
            HardwareConstants.MECANUM_KINEMATICS,
            new Rotation2d(),
            new MecanumDriveWheelPositions()
    );

    private IMU imu;
  
    /** Creates a new MecaDrive. */
    public MecaDrive(IMU imu) {
        this.imu = imu;
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public CommandBase exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce( () -> { /* one-time action goes here */ } );
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        odometry.update(imu.getRotation2d(), getCurrentWheelDistances());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(imu.getRotation2d(), getCurrentWheelDistances(), pose);
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
        if (fieldRelative) {
        driveBase.driveCartesian(xSpeed, ySpeed, rot, imu.getRotation2d());
        } else {
        driveBase.driveCartesian(xSpeed, ySpeed, rot);
        }
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        //TODO: write method
    }

    /**
     * Gets the current wheel speeds.
     *
     * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
     */
    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
            convertVelocity(frontLeftMotor.getVelocity().getValue()),
            convertVelocity(frontRightMotor.getVelocity().getValue()),
            convertVelocity(rearLeftMotor.getVelocity().getValue()),
            convertVelocity(rearRightMotor.getVelocity().getValue())
        );
    }

    /**
     * Gets the current wheel distance measurements.
     *
     * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
     */
    public MecanumDriveWheelPositions getCurrentWheelDistances() {
        // TODO: determine whether should use absolute position or just position

		return new MecanumDriveWheelPositions(convertPosition(frontLeftMotor.getPosition().getValue()), 
											  convertPosition(frontRightMotor.getPosition().getValue()),
											  convertPosition(rearLeftMotor.getPosition().getValue()),
											  convertPosition(rearRightMotor.getPosition().getValue()));
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

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        driveBase.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        imu.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return imu.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -imu.getRate();
    }
}
