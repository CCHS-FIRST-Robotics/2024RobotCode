package frc.robot.subsystems.mecaDrive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;

public class DriveIOFalcon500 implements DriveIO {
  private final TalonFX frMotor;
  private final TalonFX flMotor;
  private final TalonFX brMotor;
  private final TalonFX blMotor;

  private final AHRS imu;

  public DriveIOFalcon500() {
    frMotor = new TalonFX(1);
    flMotor = new TalonFX(2);
    brMotor = new TalonFX(3);
    blMotor = new TalonFX(4);

    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    
    currentConfigs.StatorCurrentLimitEnable = true;
    currentConfigs.StatorCurrentLimit = 40;
    config.CurrentLimits = currentConfigs;

    frMotor.getConfigurator().apply(config);
    flMotor.getConfigurator().apply(config);
    brMotor.getConfigurator().apply(config);
    blMotor.getConfigurator().apply(config);

    frMotor.setInverted(false);
    flMotor.setInverted(true);
    brMotor.setInverted(false);
    blMotor.setInverted(true);

    imu = new AHRS();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.frPositionRaw = frMotor.getPosition().getValue();
    inputs.frPositionRaw = flMotor.getPosition().getValue();
    inputs.frPositionRaw = brMotor.getPosition().getValue();
    inputs.frPositionRaw = blMotor.getPosition().getValue();

    inputs.frVelocityRaw = frMotor.getVelocity().getValue();
    inputs.flVelocityRaw = flMotor.getVelocity().getValue();
    inputs.brVelocityRaw = brMotor.getVelocity().getValue();
    inputs.blVelocityRaw = blMotor.getVelocity().getValue();
    
    inputs.gyroYawRad = imu.getYaw();
  }

  @Override
  public void setVoltage(double frVolts, double flVolts, double brVolts, double blVolts) {
    frMotor.setVoltage(frVolts);
    flMotor.setVoltage(flVolts);
    brMotor.setVoltage(brVolts);
    blMotor.setVoltage(blVolts);
  }
}
