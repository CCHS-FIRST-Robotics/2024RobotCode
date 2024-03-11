package frc.robot.subsystems.drive.mecaDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DriveIOSparkMax implements DriveIO {
    private final CANSparkMax leftLeader;
    private final CANSparkMax rightLeader;
    private final CANSparkMax leftFollower;
    private final CANSparkMax rightFollower;
    // private final RelativeEncoder leftEncoder;
    // private final RelativeEncoder rightEncoder;

    // private final Pigeon2 gyro;

    public DriveIOSparkMax() {
        leftLeader = new CANSparkMax(1, MotorType.kBrushless);
        rightLeader = new CANSparkMax(2, MotorType.kBrushless);
        leftFollower = new CANSparkMax(3, MotorType.kBrushless);
        rightFollower = new CANSparkMax(4, MotorType.kBrushless);

        // leftEncoder = leftLeader.getEncoder();
        // rightEncoder = rightLeader.getEncoder();

        leftLeader.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        leftLeader.setInverted(false);
        rightLeader.setInverted(true);
        leftFollower.follow(leftLeader, false);
        rightFollower.follow(rightLeader, false);

        leftLeader.enableVoltageCompensation(12.0);
        rightLeader.enableVoltageCompensation(12.0);
        leftLeader.setSmartCurrentLimit(30);
        rightLeader.setSmartCurrentLimit(30);

        leftLeader.burnFlash();
        rightLeader.burnFlash();
        leftFollower.burnFlash();
        rightFollower.burnFlash();

        // gyro = new Pigeon2(0);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        // inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() /
        // GEAR_RATIO);
        // inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition()
        // / GEAR_RATIO);
        // inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        // leftEncoder.getVelocity() / GEAR_RATIO);
        // inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        // rightEncoder.getVelocity() / GEAR_RATIO);
        // inputs.gyroYawRad = gyro.getYaw();
    }

    @Override
    public void setVoltage(double frVolts, double flVolts, double brVolts, double blVolts) {
        // leftLeader.setVoltage(leftVolts);
        // rightLeader.setVoltage(rightVolts);
    }
}
