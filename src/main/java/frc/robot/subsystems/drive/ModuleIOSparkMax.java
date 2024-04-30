package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.units.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.filter.MedianFilter;
import frc.robot.Constants;

public class ModuleIOSparkMax implements ModuleIO {
    /* MOTOR CONTROLLERS + PID */
    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;
    private final SparkPIDController driveSparkMaxPIDF;
    private final SparkPIDController turnSparkMaxPIDF;

    // ! lol tune these
    public double driveKp = 0.00001; // 00015
    public double driveKd = 0.0;
    public double driveKi = 0.0; // 0.000008

    public double driveKs = 0.0; // 0.19
    public double driveKv = 0.136898; // From NEO datasheet (473kV): 0.136194 V/(rad/s) -
                                      // https://www.wolframalpha.com/input?i=1%2F%28473+*+2pi%2F60%29+*+%2850.0+%2F+14.0%29+*+%2817.0+%2F+27.0%29+*+%2845.0+%2F+15.0%29
    public double driveKa = 0.020864; // 0.0148

    public double turnKp = 8;
    public double turnKd = 0.00;

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(driveKs, driveKv, driveKa); // kV
                                                                                                             // UNITS:
                                                                                                             // VOLTS /
                                                                                                             // (RAD PER
                                                                                                             // SECOND)

    /* ENCODERS */
    private final RelativeEncoder driveEncoder; // NEO Encoder
    private final RelativeEncoder turnRelativeEncoder; // NEO Encoder
    private final AbsoluteEncoder turnAbsoluteEncoder; // CANandcoder

    private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private final double turnAfterEncoderReduction = 150.0 / 7.0;
    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double couplingRatio = 50d / 14d; // equal to the first stage gear ratio

    private final MedianFilter driveVoltageFilter = new MedianFilter(1000);
    private final MedianFilter turnVoltageFilter = new MedianFilter(1000);

    private final boolean isTurnMotorInverted = true;
    // private final Rotation2d absoluteEncoderOffset;

    private Measure<Velocity<Angle>> prevVelocity = RadiansPerSecond.of(0.0);
    // private double prevVelocity = 0;

    public int index;

    /**
     * Constructs a new ModuleIOSparkMax object
     * Sets PID constants and configures the SparkMAX's + encoders
     * 
     * @param index The index of the module
     */
    public ModuleIOSparkMax(int index) {
        System.out.println("[Init] Creating ModuleIOSparkMax " + Integer.toString(index));
        this.index = index;

        driveSparkMax = new CANSparkMax(2 + 2 * index, MotorType.kBrushless);
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

        turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        turnAbsoluteEncoder.setInverted(isTurnMotorInverted);
        // absoluteEncoderOffset = new Rotation2d(-3.03887450);
        turnSparkMaxPIDF.setFeedbackDevice(turnAbsoluteEncoder);

        driveSparkMax.setCANTimeout(500);
        turnSparkMax.setCANTimeout(500);

        driveEncoder = driveSparkMax.getEncoder();
        turnRelativeEncoder = turnSparkMax.getEncoder();

        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
        turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // report absolute encoder measurements at 20ms
                                                                         // (default: 200ms)

        turnSparkMax.setInverted(isTurnMotorInverted);
        if (index == 10) {
            driveSparkMax.setInverted(true);
        } else {
            driveSparkMax.setInverted(false);
        }

        driveSparkMax.setSmartCurrentLimit(50);
        turnSparkMax.setSmartCurrentLimit(30);
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
        turnRelativeEncoder.setPositionConversionFactor(1);

        turnAbsoluteEncoder.setAverageDepth(2); // NOTE: changed from 8 since last tested run (12/15/23) -- was at 2 a
                                                // couple weeks ago tho

        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

        System.out.println("TESTING");
        System.out.println(driveSparkMax.burnFlash() == REVLibError.kOk);
        System.out.println(turnSparkMax.burnFlash() == REVLibError.kOk);
    }

    /*
     * (non-Javadoc)
     * 
     * @see
     * frc.robot.subsystems.swerveDrive.ModuleIO#updateInputs(frc.robot.subsystems.
     * swerveDrive.ModuleIO.ModuleIOInputs)
     */
    public void updateInputs(ModuleIOInputs inputs) {
        // update drive motor info
        inputs.driveRawPositionRad = // doesnt account for coupling
                Rotations.of(driveEncoder.getPosition() / driveAfterEncoderReduction);
        inputs.drivePositionRad = Rotations.of((driveEncoder.getPosition()
                + turnRelativeEncoder.getPosition() / turnAfterEncoderReduction * couplingRatio)
                / driveAfterEncoderReduction);
        inputs.driveVelocityRadPerSec = Rotations.per(Minute).of(driveEncoder.getVelocity()
                / driveAfterEncoderReduction);
        inputs.driveAppliedVolts = Volts.of(driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage());
        inputs.driveAverageBusVoltage = Volts.of(driveVoltageFilter.calculate(driveSparkMax.getBusVoltage()));
        inputs.driveCurrentAmps = Amps.of(driveSparkMax.getOutputCurrent());
        inputs.driveTempCelcius = Celsius.of(driveSparkMax.getMotorTemperature());

        // update turning motor info
        // System.out.println(turnAbsoluteEncoder.getPosition().getValue());
        inputs.turnAbsolutePositionRad = Radians.of(
                MathUtil.angleModulus(
                        new Rotation2d(
                                turnAbsoluteEncoder.getPosition() // POSITION IN ROTATIONS
                                        * 2 * Math.PI)
                                .getRadians()));
        // inputs.turnAbsolutePositionRad = turnAbsoluteEncoder.getPosition();

        inputs.turnPositionRad = Rotations.of(turnRelativeEncoder.getPosition()
                / turnAfterEncoderReduction);
        inputs.turnVelocityRadPerSec = Rotations.per(Minute).of(turnRelativeEncoder.getVelocity()
                / turnAfterEncoderReduction);
        inputs.turnAppliedVolts = Volts.of(turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage());
        inputs.turnAverageBusVoltage = Volts.of(turnVoltageFilter.calculate(turnSparkMax.getBusVoltage()));
        inputs.turnCurrentAmps = Amps.of(turnSparkMax.getOutputCurrent());
        inputs.turnTempCelcius = Celsius.of(turnSparkMax.getMotorTemperature());
    }

    /*
     * (non-Javadoc)
     * 
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#setDriveVoltage(double)
     */
    public void setDriveVoltage(Measure<Voltage> volts) {
        // driveSparkMax.setVoltage(volts.in(Volts));
    }

    /*
     * (non-Javadoc)
     * 
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#setTurnVoltage(double)
     */
    public void setTurnVoltage(Measure<Voltage> volts) {
        // turnSparkMax.setVoltage(volts.in(Volts));
    }

    /*
     * (non-Javadoc)
     * 
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#setDriveVelocity(double)
     */
    public void setDriveVelocity(Measure<Velocity<Angle>> velocity) {
        System.out.println(prevVelocity.in(RadiansPerSecond));
        System.out.println(velocity.in(RadiansPerSecond));
        System.out.println(driveFeedforward.calculate(
                        prevVelocity.in(RadiansPerSecond),
                        velocity.in(RadiansPerSecond),
                        Constants.PERIOD));
        driveSparkMaxPIDF.setReference(
                velocity.in(Rotations.per(Minute)) * driveAfterEncoderReduction,
                CANSparkMax.ControlType.kVelocity,
                0,
                driveFeedforward.calculate(
                        prevVelocity.in(RadiansPerSecond),
                        velocity.in(RadiansPerSecond),
                        Constants.PERIOD));
        prevVelocity = velocity;
    }

    /*
     * (non-Javadoc)
     * 
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#setTurnPosition(double)
     */
    public void setTurnPosition(Measure<Angle> position) {
        // Adjust from [-PI, PI] (wrapped angle, so initially -pi was 2pi) -> [0, 2PI]
        position = Radians.of(
                MathUtil.inputModulus(position.in(Radians), 0, 2 * Math.PI));

        turnSparkMaxPIDF.setReference(
                position.in(Rotations),
                CANSparkMax.ControlType.kPosition,
                0);
    }

    /*
     * (non-Javadoc)
     * 
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#setDriveBrakeMode(boolean)
     */
    public void setDriveBrakeMode(boolean enable) {
        driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    /*
     * (non-Javadoc)
     * 
     * @see frc.robot.subsystems.swerveDrive.ModuleIO#setTurnBrakeMode(boolean)
     */
    public void setTurnBrakeMode(boolean enable) {
        turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
