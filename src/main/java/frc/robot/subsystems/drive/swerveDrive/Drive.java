package frc.robot.subsystems.drive.swerveDrive;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.MutableMeasure.mutable;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Consumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.*;
import frc.robot.utils.DriveTrajectory;
import frc.robot.utils.PoseEstimator;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.utils.DriveTrajectory;
import frc.robot.utils.DriveTrajectoryGenerator;
import frc.robot.utils.PoseEstimator;


public class Drive extends SubsystemBase {

    /*
     * CONSTANTS
     */

    private static final Measure<Velocity<Distance>> coastThresholdMetersPerSec =
        MetersPerSecond.of(0.05); // Need to be under this to switch to coast when disabling
    private static final Measure<Velocity<Distance>> coastThresholdSecs =
        MetersPerSecond.of(6.0); // Need to be under the above speed for this length of time to switch to coast
    private static final Measure<Angle> ledsFallenAngle = Degrees.of(60.0); // Threshold to detect falls

    // Define Gyro IO and inputs
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    // Define Module objects
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    // Constants for the drivebase
    private static Measure<Velocity<Distance>> maxLinearSpeed = MetersPerSecond.of(4.5);
    private static final Measure<Velocity<Velocity<Distance>>> maxLinearAcceleration = MetersPerSecondPerSecond.of(9.0);
    private static final Measure<Distance> trackWidthX = Inches.of(22.5);
    private static final Measure<Distance> trackWidthY = Inches.of(22.5);
    private static final Measure<Velocity<Angle>> maxAngularSpeed = RadiansPerSecond.of(8 * Math.PI);
    private static final Measure<Velocity<Velocity<Angle>>> maxAngularAcceleration = RadiansPerSecond.per(Seconds).of(10 * Math.PI);

    // Define Kinematics object
    private SwerveDriveKinematics kinematics = getKinematics();

    /*
     * TRAJECTORIES & CONTROLS
     */

    // Initialize setpoints
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
    // Initialize trajectory info
    private Pose2d positionSetpointTrajectory = new Pose2d();
    private Twist2d twistSetpointTrajectory = new Twist2d();
    private ArrayList<Pose2d> positionTrajectory = new ArrayList<Pose2d>();
    private ArrayList<Twist2d> twistTrajectory = new ArrayList<Twist2d>();
    private int trajectoryCounter = -1;

    /* 
     * POSITIONAL CONTROL
     */
    
    // POSITION PID CONSTANTS - SHOULD NOT BE NEGATIVE
    private double kPx = 2.3; // 0.4
    private double kPy = 2.3; // 0.33
    private double kPHeading = 3; // 0.25 // 0.5

    private double kDx = 0.1; // 
    private double kDy = 0.1; // 
    private double kDHeading = .3; // 

    private double kIx = 0.1; // 0.12
    private double kIy = 0.1; // 0.15
    // private double kPlinear =
    private double kIHeading = 0.00; // 0.05

    private PIDController xController = new PIDController(kPx, kIx, kDx);
    private PIDController yController = new PIDController(kPy, kIy, kDy);
    private PIDController headingController = new PIDController(kPHeading, kIHeading, kDHeading);

    /* 
     * ODOMETRY
     */

    // Initialize estimated positions
    private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
    private Rotation2d lastGyroYaw = new Rotation2d();
    private Twist2d fieldVelocity = new Twist2d();
    private Pose2d fieldPosition = new Pose2d(); // Use poseEstimator instead
    private Pose2d rawFieldPosition = new Pose2d(); // doesnt account for coa TODO: write lol
    private PoseEstimator poseEstimator;

    /*
     * SYSID STUFF
     */

    private final MutableMeasure<Voltage> characterizationVolts = mutable(Volts.of(0));
    private final MutableMeasure<Angle> characterizationDistanceAngular = mutable(Radians.of(0));
    private final MutableMeasure<Velocity<Angle>> characterizationVelocityAngular = mutable(RadiansPerSecond.of(0));
    
    // Tell SysId how to record a frame of data for each motor on the mechanism being
    // characterized (real uses URCL, sim uses manual logging)
    Consumer<SysIdRoutineLog> log = (Constants.CURRENT_MODE == Mode.REAL) ? 
                null : log -> {
                    // Record a frame for the left motors.  Since these share an encoder, we consider
                    // the entire group to be one motor.
                    log.motor("drive-0")
                        .voltage(
                            characterizationVolts
                        )
                        .angularPosition(
                            characterizationDistanceAngular.mut_replace(modules[0].getCharacterizationPosition())
                        )
                        .angularVelocity(
                            characterizationVelocityAngular.mut_replace(modules[0].getCharacterizationVelocity())
                        );
                };
    
    SysIdRoutine characterizationRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(.8),
                Volts.of(3),
                Seconds.of(5),
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motors.
                (Measure<Voltage> volts) -> {
                    runCharacterization(volts);
                },
                log,
                this
            )
        );


    /* 
     * OTHER
     */

    int i = 0;

    private boolean isBrakeMode = true;
    private Timer lastMovementTimer = new Timer();

    // auto path
   private ArrayList<String> autoPaths;
   private int currentPathNum = 1; // 0 in the list is the first path

    // Control modes for the drive
    public enum CONTROL_MODE {
        DISABLED,
        MODULE_SETPOINT,
        CHASSIS_SETPOINT,
        POSITION_SETPOINT,
        CHARACTERIZING
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
        Logger.recordOutput("SysIdTestState", "none");

        this.gyroIO = gyroIO;

        // idk why it wont let me put it above??
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        lastMovementTimer.start();
        // whats the difference between this and modules.forEach(()=> )
        // wait thats js im stupid
        for (var module : modules) {
            module.setBrakeMode(isBrakeMode);
        }

        xController.setTolerance(.035);
        yController.setTolerance(.035);
        headingController.setTolerance(.025);
    }

    public void periodic() {
        // System.out.println("HI IM RUNNING");
        i++;
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }

        // Log measured states
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = modules[i].getState();
        }
        Logger.recordOutput("SwerveStates/Measured", measuredStates);

        /*
         * UPDATE ODOMETRY 
         */

        // Get the change in position of each module
        SwerveModulePosition[] wheelDeltas = getModuleDeltas();

        // Use kinematics to convert the change in position of each module -> change in position of the robot
        var twist = kinematics.toTwist2d(wheelDeltas);
        Logger.recordOutput("Odometry/RobotTwist", twist);

        // Use the gyro to get the change in heading of the robot, instead of the change in heading of each module
        // Gyro is likely more accurate than the modules' encoders (due to slippage, etc)
        var gyroYaw = new Rotation2d(gyroInputs.yawPosition.in(Radians));
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
            getModulePositions()
        );

        /* Update field velocity */
        // Gets the speed/angle of each module and converts it to a robot velocity
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(measuredStates);
        // Convert the robot velocity to a field velocity
        Translation2d linearFieldVelocity =
            new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                .rotateBy(getYaw());
        // Convert the field velocity to a twist (use gyro for rotation speed if connected)
        fieldVelocity =
            new Twist2d(
                linearFieldVelocity.getX(),
                linearFieldVelocity.getY(),
                gyroInputs.connected
                    ? gyroInputs.yawVelocity.in(RadiansPerSecond)
                    : chassisSpeeds.omegaRadiansPerSecond);

        // Record into "RealOutputs"
        Logger.recordOutput("Odometry/FieldVelocity", fieldVelocity);
        Logger.recordOutput("Odometry/WheelPosition", fieldPosition);
        Logger.recordOutput("Odometry/FieldPosition", getPose());

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
                // System.out.println("DISABLED");
                // Clear setpoint logs
                Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
                Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
                return;
            
            case CHARACTERIZING:
                // Run in characterization mode
                for (var module : modules) {
                    module.runCharacterization(characterizationVolts);
                }
                // System.out.println(characterizationVolts.in(Volts));
                // System.out.println("RUNNING");
                // Clear setpoint logs
                Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
                Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
                break;
            
            case POSITION_SETPOINT:
                // If we've reached the end of the trajectory, hold at the last setpoint
                if (trajectoryCounter > positionTrajectory.size() - 1) {
                    trajectoryCounter = positionTrajectory.size() - 1;
                }
                // If there's no available trajectory, don't do anything
                if (trajectoryCounter == -1) break;

                // Get the position/velocity setpoints at the current point in the trajectory
                positionSetpointTrajectory = positionTrajectory.get(trajectoryCounter);
                twistSetpointTrajectory = twistTrajectory.get(trajectoryCounter);

                // System.out.println("SETPOINTS:");
                // System.out.println(positionSetpointTrajectory);
                // System.out.println(twistSetpointTrajectory);

                // Record setpoints to "RealOutputs"
                Logger.recordOutput("Auto/FieldVelocity", new Pose2d(twistSetpointTrajectory.dx, twistSetpointTrajectory.dy, new Rotation2d(twistSetpointTrajectory.dtheta)));
                Logger.recordOutput("Auto/FieldPosition", positionSetpointTrajectory);

                // Get the PID output for the desired setpoint (output in m/s)
                double xPID = xController.calculate(getPose().getX(), positionSetpointTrajectory.getX());
                double yPID = yController.calculate(getPose().getY(), positionSetpointTrajectory.getY());
                double headingPID = headingController.calculate(getPose().getRotation().getRadians(), positionSetpointTrajectory.getRotation().getRadians());

                if (xController.atSetpoint()) xPID = 0;
                if (yController.atSetpoint()) yPID = 0;
                if (headingController.atSetpoint()) headingPID = 0;

                Logger.recordOutput("headingpidout", headingPID);
                Logger.recordOutput("rotout", twistSetpointTrajectory.dtheta + headingPID);

                // Add the PID output to the velocity setpoint
                chassisSetpoint = new ChassisSpeeds(
                    twistSetpointTrajectory.dx + xPID,
                    twistSetpointTrajectory.dy + yPID,
                    twistSetpointTrajectory.dtheta + headingPID
                );
                // Convert to robot oriented and send the updated velocity setpoint to the velocity controller 
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
                // Purpose: accounts for continuous movement along an arc instead of a discrete straight line, avoids skew
                // More detail here: https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/47
                var setpointTwist =
                new Pose2d()
                    .log(
                        new Pose2d(
                            chassisSetpoint.vxMetersPerSecond * Constants.PERIOD,
                            chassisSetpoint.vyMetersPerSecond * Constants.PERIOD,
                            new Rotation2d(chassisSetpoint.omegaRadiansPerSecond * Constants.PERIOD)));
                // Takes the twist deltas and converts them to velocities
                var adjustedSpeeds =
                    new ChassisSpeeds(
                        setpointTwist.dx / Constants.PERIOD,
                        setpointTwist.dy / Constants.PERIOD,
                        setpointTwist.dtheta / Constants.PERIOD
                    );

                lastSetpoint = adjustedSpeeds;

                // System.out.println(adjustedSpeeds);

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
                    optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
                    
                    // FOR TESTING ONLY
                    // optimizedStates[i] = new SwerveModuleState();
                }

                // Log setpoint states
                Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
                Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
                break;
            
            /* Runs each mode at the same setpoint -- only used for testing */
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
                Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
                Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
                break;
        }
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // if (i % 50 == 0) System.out.println(controlMode);
        // Since DriveWithJoysticks is the default command and MoveToPose runs once
        // Keep drive running the position trajectory unless overridden (driver sets a nonzero speed with joysticks)
        if (controlMode == CONTROL_MODE.POSITION_SETPOINT && speedsEqual(speeds, new ChassisSpeeds())) {
            return;
        }
        controlMode = CONTROL_MODE.CHASSIS_SETPOINT;
        chassisSetpoint = speeds;
    }

    public static boolean speedsEqual(ChassisSpeeds speeds, ChassisSpeeds other) {
        return (
            speeds.vxMetersPerSecond == other.vxMetersPerSecond &&
            speeds.vyMetersPerSecond == other.vyMetersPerSecond &&
            speeds.omegaRadiansPerSecond == other.omegaRadiansPerSecond
        );
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

    public void runPosition(DriveTrajectory driveTrajectory) {
        controlMode = CONTROL_MODE.POSITION_SETPOINT;
        this.positionTrajectory = driveTrajectory.positionTrajectory;
        this.twistTrajectory = driveTrajectory.velocityTrajectory;
        trajectoryCounter = 0;
    }

    public void runCharacterization(Measure<Voltage> volts) {
        controlMode = CONTROL_MODE.CHARACTERIZING;
        characterizationVolts.mut_replace(volts.in(Volts), Volts);
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

    public void setControlMode(CONTROL_MODE mode) {
        controlMode = mode;
    }

    public void setDriveMotorsBrakeMode(boolean isEnabled) {
        isBrakeMode = isEnabled;
        for (int i = 0; i < 4; i++) {
            modules[i].setBrakeMode(isBrakeMode);
        }
    }

    public void toggleDriveMotorsBrakeMode() {
        isBrakeMode = !isBrakeMode;
        for (int i = 0; i < 4; i++) {
            modules[i].setBrakeMode(isBrakeMode);
        }
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
        // return new Rotation2d(gyroInputs.yawPosition.in(Radians));
        return getPose().getRotation();
    }

    /** Returns the current yaw velocity (Z rotation) in radians per second. */
    public double getYawVelocity() {
        return gyroInputs.yawVelocity.in(RadiansPerSecond);
    }

    /** Returns the current pitch velocity (Y rotation) in radians per second. */
    public double getPitchVelocity() {
        return gyroInputs.pitchVelocity.in(RadiansPerSecond);
    }

    /** Returns the current roll velocity (X rotation) in radians per second. */
    public double getRollVelocity() {
        return gyroInputs.rollVelocity.in(RadiansPerSecond);
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        // return new Translation2d[] {
        //     new Translation2d(-trackWidthX.in(Meters) / 2.0, -trackWidthY.in(Meters) / 2.0),
        //     new Translation2d(trackWidthX.in(Meters) / 2.0, -trackWidthY.in(Meters) / 2.0),
        //     new Translation2d(trackWidthX.in(Meters) / 2.0, trackWidthY.in(Meters) / 2.0),
        //     new Translation2d(-trackWidthX.in(Meters) / 2.0, trackWidthY.in(Meters) / 2.0)
        // };

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
          wheelDeltas[i] =
              new SwerveModulePosition(
                  (modules[i].getPositionMeters() - lastModulePositionsMeters[i]),
                  modules[i].getAngle());
          lastModulePositionsMeters[i] = modules[i].getPositionMeters();
        }
        return wheelDeltas;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] wheelPos = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
          wheelPos[i] =
              new SwerveModulePosition(
                  (modules[i].getPositionMeters()),
                  modules[i].getAngle());
        }
        return wheelPos;
    }

    public SwerveDriveKinematics getKinematics() {
        if (kinematics != null) return kinematics;
        return new SwerveDriveKinematics(getModuleTranslations());
    }

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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return characterizationRoutine.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return characterizationRoutine.dynamic(direction);
    }

    public Command sysIdFull() {
        return characterizationRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(characterizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(characterizationRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(characterizationRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }
    
    public Command followTrajectory(DriveTrajectory traj) {
        return runOnce(
                () -> {
                    System.out.println("recording pos traj");
                    Logger.recordOutput("Auto/GeneratedTrajectory", traj.positionTrajectory.toArray(new Pose2d[traj.positionTrajectory.size()]));
                    runPosition(traj);
                }
            );
    }


    /* added for auto stuff - not good but maybe better? */
    public Command followTrajectory(ArrayList<String> path) {
        return runOnce(
                () -> {
                    DriveTrajectory traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path.get(currentPathNum));
                    System.out.println("recording pos traj");
                    Logger.recordOutput("Auto/GeneratedTrajectory", traj.positionTrajectory.toArray(new Pose2d[traj.positionTrajectory.size()]));
                    currentPathNum++;
                    runPosition(traj);
                }
            );
    }
}