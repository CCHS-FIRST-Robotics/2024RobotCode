package frc.robot.subsystems.swerveDrive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private static final double maxLinearAcceleration = 50.0;
    private static final double trackWidthX = 0;
    private static final double trackWidthY = 0;

    private double maxAngularSpeed;
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

    private ChassisSpeeds chassisSetpoint = new ChassisSpeeds();
    private SwerveModuleState moduleSetpoint = new SwerveModuleState();
    private SwerveModuleState[] lastSetpointStates =
        new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };

    
    private boolean isBrakeMode = false;
    private Timer lastMovementTimer = new Timer();

    private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
    private Rotation2d lastGyroYaw = new Rotation2d();

    enum CONTROL_MODE {
      DISABLED,
      MODULE_SETPOINT,
      CHASSIS_SETPOINT
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

            case CHASSIS_SETPOINT:
                // Convert from a field-oriented setpoint to a robot-oriented twist to achieve that setpoint
                // Brief explanation here: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/transformations.html
                // For more detail, see chapter 10 here: https://file.tavsys.net/control/controls-engineering-in-frc.pdf
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
                        setpointTwist.dtheta / Constants.PERIOD);
                // uses the IK to convert from chassis velocities to individual swerve positions/velocities
                SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(adjustedSpeeds);
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
              lastSetpointStates[i].speedMetersPerSecond, getModuleTranslations()[i].getAngle());
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
      new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
      new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0),
      new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
      new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0)
    };
  }
}
