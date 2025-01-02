package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.*;
import frc.robot.utils.DriveTrajectory;
import frc.robot.utils.PoseEstimator;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    // Define Module objects
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    // constants
    private static final Measure<Distance> trackWidthX = Inches.of(22.5);
    private static final Measure<Distance> trackWidthY = Inches.of(22.5);
    private static final Measure<Velocity<Distance>> maxLinearSpeed = MetersPerSecond.of(4.5);
    private static final Measure<Velocity<Velocity<Distance>>> maxLinearAcceleration = MetersPerSecondPerSecond.of(9.0);
    private static final Measure<Velocity<Angle>> maxAngularSpeed = RadiansPerSecond.of(8 * Math.PI);
    private static final Measure<Velocity<Velocity<Angle>>> maxAngularAcceleration = RadiansPerSecond.per(Seconds).of(10 * Math.PI);

    // ! look at this
    private SwerveDriveKinematics kinematics = getKinematics();

    /*
     * TRAJECTORIES & CONTROLS
     */
    private ChassisSpeeds chassisSetpoint = new ChassisSpeeds();
    private SwerveModuleState[] lastSetpointStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };

    // position control
    private Pose2d positionSetpointTrajectory = new Pose2d();
    private Twist2d twistSetpointTrajectory = new Twist2d();
    private ArrayList<Pose2d> positionTrajectory = new ArrayList<Pose2d>();
    private ArrayList<Twist2d> twistTrajectory = new ArrayList<Twist2d>();
    private int trajectoryCounter = -1;

    // position control
    private double kPx = 2.7;
    private double kIx = 0.05;
    private double kDx = 0.12;
    private double kPy = 2.7;
    private double kIy = 0.05;
    private double kDy = 0.12;
    private double kPHeading = 3;
    private double kIHeading = 0;
    private double kDHeading = .3;
    private PIDController xController = new PIDController(kPx, kIx, kDx);
    private PIDController yController = new PIDController(kPy, kIy, kDy);
    private PIDController headingController = new PIDController(kPHeading, kIHeading, kDHeading);

    /*
     * ODOMETRY
     */
    private PoseEstimator poseEstimator;
    private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };
    private Rotation2d lastGyroYaw = new Rotation2d();
    private Twist2d fieldVelocity = new Twist2d();
    private Pose2d fieldPosition = new Pose2d(); // Use poseEstimator instead

    /*
     * OTHER
     */
    boolean openLoop = false;

    public enum CONTROL_MODE {
        DISABLED,
        CHASSIS_SETPOINT,
        POSITION_SETPOINT,
    };
    CONTROL_MODE controlMode = CONTROL_MODE.DISABLED;

    public Drive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO
    ) {
        this.gyroIO = gyroIO;

        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);
        
        for (Module module : modules) {
            module.setBrakeMode(true);
        }
        
        xController.setTolerance(.035);
        yController.setTolerance(.035);
        headingController.setTolerance(.025);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Gyro", gyroInputs);
        for (Module module : modules) {
            module.periodic();
        }

        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = modules[i].getState();
        }

        /*
         * UPDATE ODOMETRY
         */
        SwerveModulePosition[] wheelDeltas = getModuleDeltas();

        // Use kinematics to convert the change in position of each module -> change in
        // position of the robot
        Twist2d twist = kinematics.toTwist2d(wheelDeltas);

        // Use the gyro to get the change in heading of the robot, instead of the change
        // in heading of each module
        // Gyro is likely more accurate than the modules' encoders (due to slippage,
        // etc)
        Rotation2d gyroYaw = new Rotation2d(gyroInputs.yawPosition.in(Radians));
        if (gyroInputs.connected) {
            twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
        }
        lastGyroYaw = gyroYaw;

        // Update pose estimator with the new data
        fieldPosition = fieldPosition.exp(twist);
        // poseEstimator.addOdometryData(twist, Timer.getFPGATimestamp());
        poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                (gyroInputs.connected ? gyroYaw : fieldPosition.getRotation()),
                getModulePositions());

        /* Update field velocity */
        // Gets the speed/angle of each module and converts it to a robot velocity
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(measuredStates);
        // Convert the robot velocity to a field velocity
        Translation2d linearFieldVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond)
                .rotateBy(getYaw());
        // Convert the field velocity to a twist (use gyro for rotation speed if
        // connected)
        fieldVelocity = new Twist2d(
                linearFieldVelocity.getX(),
                linearFieldVelocity.getY(),
                gyroInputs.connected
                        ? gyroInputs.yawVelocity.in(RadiansPerSecond)
                        : chassisSpeeds.omegaRadiansPerSecond);

        // Record into "RealOutputs"
        Logger.recordOutput("Odometry/FieldVelocity", fieldVelocity);
        Logger.recordOutput("Odometry/WheelPosition", fieldPosition);
        Logger.recordOutput("Odometry/FieldPosition", getPose());

        // disable the robot
        if (DriverStation.isDisabled()) {
            controlMode = CONTROL_MODE.DISABLED;
        }

        switch (controlMode) {
            case DISABLED:
                for (var module : modules) {
                    module.stop();
                }
                Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
                Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
                return;
            case POSITION_SETPOINT:
                if (trajectoryCounter == -1){ // if no available trajectory
                    break;
                }
                if (trajectoryCounter > positionTrajectory.size() - 1) { // if at the end of trajectory, hold the last setpoint
                    trajectoryCounter = positionTrajectory.size() - 1;
                }

                positionSetpointTrajectory = positionTrajectory.get(trajectoryCounter);
                twistSetpointTrajectory = twistTrajectory.get(trajectoryCounter);

                // Record setpoints to "RealOutputs"
                Logger.recordOutput("Auto/FieldVelocity", new Pose2d(twistSetpointTrajectory.dx,
                        twistSetpointTrajectory.dy, new Rotation2d(twistSetpointTrajectory.dtheta)));
                Logger.recordOutput("Auto/FieldPosition", positionSetpointTrajectory);

                // Get the PID output for the desired setpoint (output in m/s)
                // gets pid for x, y, and heading
                double xPID = xController.calculate(getPose().getX(), positionSetpointTrajectory.getX());
                double yPID = yController.calculate(getPose().getY(), positionSetpointTrajectory.getY());
                double headingPID = headingController.calculate(getPose().getRotation().getRadians(),
                    positionSetpointTrajectory.getRotation().getRadians()
                );

                if (xController.atSetpoint()){
                    xPID = 0;
                }
                if (yController.atSetpoint()){
                    yPID = 0;
                }
                if (headingController.atSetpoint()){
                    headingPID = 0;
                }

                Logger.recordOutput("headingpidout", headingPID);
                Logger.recordOutput("rotout", twistSetpointTrajectory.dtheta + headingPID);

                // add the PID output to the velocity setpoint
                chassisSetpoint = new ChassisSpeeds(
                        twistSetpointTrajectory.dx + xPID,
                        twistSetpointTrajectory.dy + yPID,
                        twistSetpointTrajectory.dtheta + headingPID);
                // Convert to robot oriented and send the updated velocity setpoint to the
                // velocity controller
                chassisSetpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                        chassisSetpoint.vxMetersPerSecond,
                        chassisSetpoint.vyMetersPerSecond,
                        chassisSetpoint.omegaRadiansPerSecond,
                        getYaw());

                trajectoryCounter++;
                // fallthrough to CHASSIS_SETPOINT, no break statement
            case CHASSIS_SETPOINT:
                // Brief explanation here:
                // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/transformations.html
                // For more detail, see chapter 10 here:
                // https://file.tavsys.net/control/controls-engineering-in-frc.pdf
                // Purpose: accounts for continuous movement along an arc instead of a discrete
                // straight line, avoids skew
                // More detail here:
                // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/47
                Twist2d setpointTwist = new Pose2d().log( // ! why use a new Pose2d every time
                    new Pose2d(
                        chassisSetpoint.vxMetersPerSecond * Constants.PERIOD,
                        chassisSetpoint.vyMetersPerSecond * Constants.PERIOD,
                        new Rotation2d(chassisSetpoint.omegaRadiansPerSecond * Constants.PERIOD)
                    )
                );
                // Takes the twist deltas and converts them to velocities
                ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(
                    setpointTwist.dx / Constants.PERIOD,
                    setpointTwist.dy / Constants.PERIOD,
                    setpointTwist.dtheta / Constants.PERIOD
                );

                // Uses the IK to convert from chassis velocities to individual swerve module positions/velocities
                Logger.recordOutput("Target Velocity", adjustedSpeeds);
                SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(adjustedSpeeds);

                // Ensure a module isnt trying to go faster than max
                SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearSpeed);

                // Set to last angles if zero
                if (adjustedSpeeds.vxMetersPerSecond == 0.0
                        && adjustedSpeeds.vyMetersPerSecond == 0.0
                        && adjustedSpeeds.omegaRadiansPerSecond == 0) {
                    for (int i = 0; i < 4; i++) {
                        setpointStates[i] = new SwerveModuleState(0.0, lastSetpointStates[i].angle);
                    }
                }

                lastSetpointStates = setpointStates;

                // Send setpoints to modules
                SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
                for (int i = 0; i < 4; i++) {
                    if (openLoop){
                        setpointStates[i].speedMetersPerSecond *= 1d / (maxLinearSpeed.in(MetersPerSecond));
                    }
                    optimizedStates[i] = modules[i].runSetpoint(setpointStates[i], openLoop);
                }

                Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
                Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
                break;
        }
    }

    public void runVelocity(ChassisSpeeds speeds) {
        // Since DriveWithJoysticks is the default command and MoveToPose runs once
        // Keep drive running the position trajectory unless overridden (driver sets a
        // nonzero speed with joysticks)
        if (controlMode == CONTROL_MODE.POSITION_SETPOINT && speedsEqual(speeds, new ChassisSpeeds())) {
            return;
        }
        controlMode = CONTROL_MODE.CHASSIS_SETPOINT;
        chassisSetpoint = speeds;
    }

    // ! check if this is necessary
    public static boolean speedsEqual(ChassisSpeeds speeds, ChassisSpeeds other) {
        return (speeds.vxMetersPerSecond == other.vxMetersPerSecond &&
                speeds.vyMetersPerSecond == other.vyMetersPerSecond &&
                speeds.omegaRadiansPerSecond == other.omegaRadiansPerSecond);
    }

    public void runPosition(DriveTrajectory driveTrajectory) {
        controlMode = CONTROL_MODE.POSITION_SETPOINT;
        this.positionTrajectory = driveTrajectory.positionTrajectory;
        this.twistTrajectory = driveTrajectory.velocityTrajectory;
        trajectoryCounter = 0;
    }

    public void setOpenLoop(boolean isOpenLoop) {
        openLoop = isOpenLoop;
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        stop();
        for (int i = 0; i < 4; i++) {
            lastSetpointStates[i] = new SwerveModuleState(
                    lastSetpointStates[i].speedMetersPerSecond, getModuleTranslations()[i].getAngle());
        }
    }

    public void setControlMode(CONTROL_MODE mode) {
        controlMode = mode;
    }

    /** Returns the maximum linear speed in meters per sec. */
    public Measure<Velocity<Distance>> getMaxLinearSpeed() {
        return maxLinearSpeed;
    }

    /** Returns the maximum linear acceleration in meters per sec per sec. */
    public Measure<Velocity<Velocity<Distance>>> getMaxLinearAcceleration() {
        return maxLinearAcceleration;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public Measure<Velocity<Angle>> getMaxAngularSpeed() {
        return maxAngularSpeed;
    }

    /** Returns the maximum angular acceleration in radians per sec. */
    public Measure<Velocity<Velocity<Angle>>> getMaxAngularAcceleration() {
        return maxAngularAcceleration;
    }

    /** Returns the current pitch (Y rotation). */
    public Rotation2d getPitch() {
        return new Rotation2d(gyroInputs.pitchPosition.in(Radians));
    }

    /** Returns the current roll (X rotation). */
    public Rotation2d getRoll() {
        return new Rotation2d(gyroInputs.rollPosition.in(Radians));
    }

    /** Returns the current yaw (Z rotation). */
    public Rotation2d getYaw() {
        return getPose().getRotation();
    }

    public Rotation2d getYawWithAllianceRotation() {
        // make field relative to red if on red team
        return getYaw().plus(
            DriverStation.getAlliance().get() == Alliance.Red ? 
            new Rotation2d(Math.PI) : 
            new Rotation2d(0)
        ); 
    }

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
                new Translation2d(-trackWidthX.in(Meters) / 2.0, -trackWidthY.in(Meters) / 2.0),
                new Translation2d(trackWidthX.in(Meters) / 2.0, -trackWidthY.in(Meters) / 2.0),
                new Translation2d(trackWidthX.in(Meters) / 2.0, trackWidthY.in(Meters) / 2.0),
                new Translation2d(-trackWidthX.in(Meters) / 2.0, trackWidthY.in(Meters) / 2.0)
        };
    }

    public SwerveModulePosition[] getModuleDeltas() {
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                    (modules[i].getPositionMeters() - lastModulePositionsMeters[i]),
                    modules[i].getAngle());
            lastModulePositionsMeters[i] = modules[i].getPositionMeters();
        }
        return wheelDeltas;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] wheelPos = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelPos[i] = new SwerveModulePosition(
                    (modules[i].getPositionMeters()),
                    modules[i].getAngle());
        }
        return wheelPos;
    }

    public SwerveDriveKinematics getKinematics() {
        if (kinematics != null){

            return kinematics;
        }
        return new SwerveDriveKinematics(getModuleTranslations());
    }

    // ! are these seriously needed
    public PIDController getXController() {
        return xController;
    }

    public PIDController getYController() {
        return yController;
    }

    public PIDController getHeadingController() {
        return headingController;
    }

    public void setPoseEstimator(PoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    public Pose2d getPose() {
        return poseEstimator.getPoseEstimate();
    }

    public Twist2d getVelocity() {
        return fieldVelocity;
    }

    // ! figure out how this works
    public Command followTrajectory(DriveTrajectory traj) {
        return runOnce(
            () -> {
                Logger.recordOutput("Auto/GeneratedTrajectory", traj.positionTrajectory.toArray(new Pose2d[traj.positionTrajectory.size()]));
                runPosition(traj);
            }
        );
    }
}