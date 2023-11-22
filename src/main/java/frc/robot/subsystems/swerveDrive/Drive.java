package frc.robot.subsystems.swerveDrive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;


public class Drive extends SubsystemBase {
    private static final double coastThresholdMetersPerSec =
        0.05; // Need to be under this to switch to coast when disabling
    private static final double coastThresholdSecs =
        6.0; // Need to be under the above speed for this length of time to switch to coast
    private static final double ledsFallenAngleDegrees = 60.0; // Threshold to detect falls

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private static final double maxLinearSpeed = 4.5;
    private static final double maxLinearAcceleration = 8.0;
    private static final double trackWidthX = Units.inchesToMeters(22.5);
    private static final double trackWidthY = Units.inchesToMeters(22.5);

    private double maxAngularSpeed = 4 * Math.PI;
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

    private ChassisSpeeds chassisSetpoint = new ChassisSpeeds();
    private ChassisSpeeds lastSetpoint = new ChassisSpeeds();
    private SwerveModuleState moduleSetpoint = new SwerveModuleState();
    private SwerveModuleState[] lastSetpointStates =
        new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };
    private Pose2d positionSetpointTrajectory = new Pose2d();
    private Twist2d twistSetpointTrajectory = new Twist2d();
    private ArrayList<Pose2d> positionTrajectory = new ArrayList<Pose2d>();
    private ArrayList<Twist2d> twistTrajectory = new ArrayList<Twist2d>();
    private int trajectoryCounter = -1;
    
    // POSITION PID CONSTANTS
    private double kPx = 0.0; // 0.5
    private double kPy = 0.0; // 0.5
    private double kPHeading = 0.0; // 0.5

    private double kIx = 0.0; // 0.3
    private double kIy = 0.0; // 0.3
    private double kIHeading = 0.0; // 0.3

    private PIDController xController = new PIDController(kPx, kIx, 0.0);
    private PIDController yController = new PIDController(kPy, kIy, 0.0);
    private PIDController headingController = new PIDController(kPHeading, kIHeading, 0.0);

    private boolean isBrakeMode = false;
    private Timer lastMovementTimer = new Timer();

    private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
    private Rotation2d lastGyroYaw = new Rotation2d();
    private Twist2d fieldVelocity = new Twist2d();
    private Pose2d fieldPosition = new Pose2d();

    enum CONTROL_MODE {
        DISABLED,
        MODULE_SETPOINT,
        CHASSIS_SETPOINT,
        POSITION_SETPOINT
    };

    CONTROL_MODE controlMode = CONTROL_MODE.DISABLED;

    public Drive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO
    ) {
        System.out.println("[Init] Creating Drive");
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);
        lastMovementTimer.start();
        // whats the difference between this and modules.forEach(()=> )
        // wait thats js im stupid
        for (var module : modules) {
            module.setBrakeMode(false);
        }
    }

    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }

        // Log measured states
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = modules[i].getState();
        }
        Logger.getInstance().recordOutput("SwerveStates/Measured", measuredStates);


        // Update odometry
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
          wheelDeltas[i] =
              new SwerveModulePosition(
                  (modules[i].getPositionMeters() - lastModulePositionsMeters[i]),
                  modules[i].getAngle());
          lastModulePositionsMeters[i] = modules[i].getPositionMeters();
        }
        var twist = kinematics.toTwist2d(wheelDeltas);
        var gyroYaw = new Rotation2d(gyroInputs.yawPositionRad);
        if (gyroInputs.connected) {
            twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
        }
        lastGyroYaw = gyroYaw;
        fieldPosition = fieldPosition.exp(twist);

        // Update field velocity
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(measuredStates);
        Translation2d linearFieldVelocity =
            new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                .rotateBy(getYaw());
        fieldVelocity =
            new Twist2d(
                linearFieldVelocity.getX(),
                linearFieldVelocity.getY(),
                gyroInputs.connected
                    ? gyroInputs.yawVelocityRadPerSec
                    : chassisSpeeds.omegaRadiansPerSecond);

        Logger.getInstance().recordOutput("Odometry/FieldVelocity", new Pose2d(fieldVelocity.dx, fieldVelocity.dy, new Rotation2d(fieldVelocity.dtheta)));
        Logger.getInstance().recordOutput("Odometry/FieldPosition", fieldPosition);

        if (DriverStation.isDisabled()) {
            controlMode = CONTROL_MODE.DISABLED;
        }

        // Run modules
        switch (controlMode) {
            case DISABLED:
                // Stop moving while disabled
                for (var module : modules) {
                    module.stop();
                }

                // Clear setpoint logs
                Logger.getInstance().recordOutput("SwerveStates/Setpoints", new double[] {});
                Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", new double[] {});
                return;

            case POSITION_SETPOINT:
                if (trajectoryCounter == -1) break;
                if (trajectoryCounter > positionTrajectory.size() - 1) {
                    trajectoryCounter = positionTrajectory.size() - 1;
                }

                positionSetpointTrajectory = positionTrajectory.get(trajectoryCounter);
                twistSetpointTrajectory = twistTrajectory.get(trajectoryCounter);

                System.out.println("SETPOINTS:");
                System.out.println(positionSetpointTrajectory);
                System.out.println(twistSetpointTrajectory);

                Logger.getInstance().recordOutput("Auto/FieldVelocity", new Pose2d(twistSetpointTrajectory.dx, twistSetpointTrajectory.dy, new Rotation2d(twistSetpointTrajectory.dtheta)));
                Logger.getInstance().recordOutput("Auto/FieldPosition", positionSetpointTrajectory);

                double xPID = xController.calculate(getPose().getX(), positionSetpointTrajectory.getX());
                double yPID = yController.calculate(getPose().getY(), positionSetpointTrajectory.getY());
                double headingPID = headingController.calculate(getPose().getRotation().getRadians(), positionSetpointTrajectory.getRotation().getRadians());

                chassisSetpoint = new ChassisSpeeds(
                    twistSetpointTrajectory.dx + xPID,
                    twistSetpointTrajectory.dy + yPID,
                    twistSetpointTrajectory.dtheta + headingPID
                );
                chassisSetpoint =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        chassisSetpoint.vxMetersPerSecond,
                        chassisSetpoint.vyMetersPerSecond,
                        chassisSetpoint.omegaRadiansPerSecond,
                        getYaw()
                    );
                
                trajectoryCounter++;
                /*
                 * WARNING:
                 * NO BREAK HERE
                 * IT CONTINUES TO CHASSIS_SETPOINT CASE
                 */
                // fallthrough
            case CHASSIS_SETPOINT:
                // Brief explanation here: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/transformations.html
                // For more detail, see chapter 10 here: https://file.tavsys.net/control/controls-engineering-in-frc.pdf
                // Purpose: accounts for movement along an arc instead of a straight line, avoids skew
                // More detail here: https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/47
                var setpointTwist =
                new Pose2d()
                    .log(
                        new Pose2d(
                            chassisSetpoint.vxMetersPerSecond * Constants.PERIOD,
                            chassisSetpoint.vyMetersPerSecond * Constants.PERIOD,
                            new Rotation2d(chassisSetpoint.omegaRadiansPerSecond * Constants.PERIOD)));
                // takes the twist deltas and converts them to velocities
                var adjustedSpeeds =
                    new ChassisSpeeds(
                        setpointTwist.dx / Constants.PERIOD,
                        setpointTwist.dy / Constants.PERIOD,
                        setpointTwist.dtheta / Constants.PERIOD
                    );

                // desaturate speeds if above the max acceleration
                // TODO: I think I already do this in the command... oml
                double[] accelVector = {
                    (adjustedSpeeds.vxMetersPerSecond - lastSetpoint.vxMetersPerSecond) / Constants.PERIOD,
                    (adjustedSpeeds.vyMetersPerSecond - lastSetpoint.vyMetersPerSecond) / Constants.PERIOD
                };
                double acceleration = Math.sqrt(
                    accelVector[0]*accelVector[0] + 
                    accelVector[1]*accelVector[1]
                );
                double accelDir = Math.atan2(accelVector[1], accelVector[0]);
                if (acceleration > maxLinearAcceleration) {
                    adjustedSpeeds.vxMetersPerSecond = lastSetpoint.vxMetersPerSecond + Math.cos(accelDir) * maxLinearAcceleration * Constants.PERIOD;
                    adjustedSpeeds.vyMetersPerSecond = lastSetpoint.vyMetersPerSecond + Math.sin(accelDir) * maxLinearAcceleration * Constants.PERIOD;
                }
                lastSetpoint = adjustedSpeeds;

                // System.out.println(adjustedSpeeds);

                // uses the IK to convert from chassis velocities to individual swerve positions/velocities
                SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(adjustedSpeeds);
                // System.out.println(setpointStates[0]);
                // ensure a module isnt trying to go faster than max
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
                    optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
                }

                // Log setpoint states
                Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);
                Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
                break;

            case MODULE_SETPOINT:
                setpointStates = new SwerveModuleState[] {moduleSetpoint, moduleSetpoint, moduleSetpoint, moduleSetpoint};
                lastSetpointStates = setpointStates;
                // System.out.println(moduleSetpoint.angle.getDegrees());

                // Send setpoints to modules
                optimizedStates = new SwerveModuleState[4];
                for (int i = 0; i < 4; i++) {
                    optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
                }

                // Log setpoint states
                Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);
                Logger.getInstance().recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
                break;
        }
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        controlMode = CONTROL_MODE.CHASSIS_SETPOINT;
        chassisSetpoint = speeds;
    }

    public void runModules(SwerveModuleState setpoint) {
        controlMode = CONTROL_MODE.MODULE_SETPOINT;
        moduleSetpoint = setpoint;
    }

    public void runPosition(ArrayList<Pose2d> poseTrajectory, ArrayList<Twist2d> twistTrajectory) {
        controlMode = CONTROL_MODE.POSITION_SETPOINT;
        this.positionTrajectory = poseTrajectory;
        this.twistTrajectory = twistTrajectory;
        trajectoryCounter = 0;
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        stop();
        for (int i = 0; i < 4; i++) {
            lastSetpointStates[i] =
                new SwerveModuleState(
                    lastSetpointStates[i].speedMetersPerSecond, getModuleTranslations()[i].getAngle()
                );
        }
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxLinearSpeed;
    }

    /** Returns the maximum linear acceleration in meters per sec per sec. */
    public double getMaxLinearAccelerationMetersPerSecPerSec() {
        return maxLinearAcceleration;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return maxAngularSpeed;
    }

    /** Returns the current pitch (Y rotation). */
    public Rotation2d getPitch() {
        return new Rotation2d(gyroInputs.pitchPositionRad);
    }

    /** Returns the current roll (X rotation). */
    public Rotation2d getRoll() {
        return new Rotation2d(gyroInputs.rollPositionRad);
    }

    /** Returns the current yaw (Z rotation). */
    public Rotation2d getYaw() {
        return new Rotation2d(gyroInputs.yawPositionRad);
    }

    /** Returns the current yaw velocity (Z rotation) in radians per second. */
    public double getYawVelocity() {
        return gyroInputs.yawVelocityRadPerSec;
    }

    /** Returns the current pitch velocity (Y rotation) in radians per second. */
    public double getPitchVelocity() {
        return gyroInputs.pitchVelocityRadPerSec;
    }

    /** Returns the current roll velocity (X rotation) in radians per second. */
    public double getRollVelocity() {
        return gyroInputs.rollVelocityRadPerSec;
    }

    /** Returns an array of module translations. */
    public Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
            new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
            new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0),
            new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0)
        };
    }

    public Pose2d getPose() {
        return fieldPosition;
    } 

    public Twist2d getVelocity() {
        return fieldVelocity;
    }
}
