package frc.robot.subsystems.swerveDrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

public class ModuleIOSparkMax implements ModuleIO {
    /* MOTOR CONTROLLERS + PID */
    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;

    private final SparkMaxPIDController driveSparkMaxPIDF;
    private final SparkMaxPIDController turnSparkMaxPIDF;

    // TODO: update constants in periodic once tunable is set up
    private static final double driveKp = 0.00017; 
    private static final double driveKd = 0.0;
    private static final double driveKi = 0.000000; // 0.000008
    private static final double driveKs = 0.0;
    private static final double driveKv = 0.17; 

    private static final double turnKp = 5; 
    private static final double turnKd = 0.00;

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(driveKs, driveKv); // kV UNITS: VOLTS / (RAD PER SECOND)

    /* ENCODERS */
    private final RelativeEncoder driveEncoder; // NEO Encoder
    private final RelativeEncoder turnRelativeEncoder; // NEO Encoder
    private final AbsoluteEncoder turnAbsoluteEncoder; // CANandcoder

    private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private final double turnAfterEncoderReduction = 150.0 / 7.0;

    private final boolean isTurnMotorInverted = false;
    // private final Rotation2d absoluteEncoderOffset;

    int index;

    /**
     * Constructs a new ModuleIOSparkMax object
     * Sets PID constants and configures the SparkMAX's + encoders
     * 
     * @param index The index of the module
     */
    public ModuleIOSparkMax(int index) {
        System.out.println("[Init] Creating ModuleIOSparkMax " + Integer.toString(index));
        this.index = index;

        driveSparkMax = new CANSparkMax(2 + 2 *index, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(1 + 2 * index, MotorType.kBrushless);

        driveSparkMaxPIDF = driveSparkMax.getPIDController();
        turnSparkMaxPIDF = turnSparkMax.getPIDController();

        driveSparkMaxPIDF.setP(driveKp, 0);
        driveSparkMaxPIDF.setI(driveKi, 0);
        driveSparkMaxPIDF.setD(driveKd, 0);
        driveSparkMaxPIDF.setFF(0, 0);

        turnSparkMaxPIDF.setP(turnKp, 0);
        turnSparkMaxPIDF.setD(turnKd, 0);
        turnSparkMaxPIDF.setI(0, 0);
        turnSparkMaxPIDF.setFF(0, 0);

        turnSparkMaxPIDF.setPositionPIDWrappingEnabled(true);
        turnSparkMaxPIDF.setPositionPIDWrappingMinInput(0);
        turnSparkMaxPIDF.setPositionPIDWrappingMaxInput(1);

        turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        // absoluteEncoderOffset = new Rotation2d(-3.03887450);
        turnSparkMaxPIDF.setFeedbackDevice(turnAbsoluteEncoder);

        driveSparkMax.setCANTimeout(500);
        turnSparkMax.setCANTimeout(500);

        driveEncoder = driveSparkMax.getEncoder();
        turnRelativeEncoder = turnSparkMax.getEncoder();

        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
        turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // report absolute encoder measurements at 20ms (default: 200ms)

        turnSparkMax.setInverted(isTurnMotorInverted);
        if (index == 2 || index == 3) {
            driveSparkMax.setInverted(false);
        } else {
            driveSparkMax.setInverted(true);
        }

        driveSparkMax.setSmartCurrentLimit(60);
        turnSparkMax.setSmartCurrentLimit(40);
        driveSparkMax.enableVoltageCompensation(12.0);
        turnSparkMax.enableVoltageCompensation(12.0);

        driveSparkMax.setIdleMode(IdleMode.kBrake);
        turnSparkMax.setIdleMode(IdleMode.kBrake);

        driveEncoder.setPosition(0.0);
        driveEncoder.setMeasurementPeriod(10);
        driveEncoder.setAverageDepth(2);

        turnRelativeEncoder.setPosition(0.0);
        turnRelativeEncoder.setMeasurementPeriod(10);
        turnRelativeEncoder.setAverageDepth(2);

        // TODO: any other params/tuning?
        turnAbsoluteEncoder.setAverageDepth(8); // NOTE: changed from 2 since last tested run (12/3/23)

        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);


        // System.out.println("TESTING");
        // System.out.println(driveSparkMax.burnFlash() == REVLibError.kOk);
        // System.out.println(turnSparkMax.burnFlash() == REVLibError.kOk);
    }

    /* (non-Javadoc)
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#updateInputs(frc.robot.subsystems.swerveDrive.ModuleIO.ModuleIOInputs)
     */
    public void updateInputs(ModuleIOInputs inputs) {
        // update drive motor info
        inputs.drivePositionRad =
            Units.rotationsToRadians(driveEncoder.getPosition()) / driveAfterEncoderReduction;
        inputs.driveVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
                / driveAfterEncoderReduction;
        inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
        inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
        inputs.driveTempCelcius = new double[] {driveSparkMax.getMotorTemperature()};

        // update turning motor info
        // System.out.println(turnAbsoluteEncoder.getPosition().getValue());
        inputs.turnAbsolutePositionRad =
            MathUtil.angleModulus(
                new Rotation2d(
                        turnAbsoluteEncoder.getPosition() // POSITION IN ROTATIONS
                        * 2 * Math.PI)
                    .getRadians());
        // TODO: FOR TESTING ONLY:
        // inputs.turnAbsolutePositionRad = turnAbsoluteEncoder.getPosition();

        inputs.turnPositionRad =
            Units.rotationsToRadians(turnRelativeEncoder.getPosition())
                / turnAfterEncoderReduction;
        inputs.turnVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
                / turnAfterEncoderReduction;
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
        inputs.turnTempCelcius = new double[] {turnSparkMax.getMotorTemperature()};
    }

    /* (non-Javadoc)
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#setDriveVoltage(double)
     */
    public void setDriveVoltage(double volts) {
        driveSparkMax.setVoltage(volts);
    }

    /* (non-Javadoc)
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#setTurnVoltage(double)
     */
    public void setTurnVoltage(double volts) {
        turnSparkMax.setVoltage(volts);
    }

    /* (non-Javadoc)
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#setDriveVelocity(double)
     */
    public void setDriveVelocity(double velocityRadPerSec) {
        double motorRPM = Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * driveAfterEncoderReduction;

        driveSparkMaxPIDF.setReference(
            motorRPM,
            CANSparkMax.ControlType.kVelocity,
            0,
            driveFeedforward.calculate(velocityRadPerSec)
        );
    }

    /* (non-Javadoc)
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#setTurnPosition(double)
     */
    public void setTurnPosition(double positionRad) {
        // Adjust from [-PI, PI] (wrapped angle, so initially -pi was 2pi) -> [0, 2PI] 
        double positionRotations = Units.radiansToRotations(
            MathUtil.inputModulus(positionRad, 0, 2 * Math.PI)
        );
        turnSparkMaxPIDF.setReference(
            positionRotations,
            CANSparkMax.ControlType.kPosition,
            0
        );
    }

    /* (non-Javadoc)
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#setDriveBrakeMode(boolean)
     */
    public void setDriveBrakeMode(boolean enable) {
        driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    /* (non-Javadoc)
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#setTurnBrakeMode(boolean)
     */
    public void setTurnBrakeMode(boolean enable) {
        turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
