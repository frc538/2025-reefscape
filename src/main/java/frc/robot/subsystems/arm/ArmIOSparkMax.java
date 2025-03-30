package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class ArmIOSparkMax implements ArmIO {
  private final SparkMax mSparkMax;

  SparkMaxConfig mConfig = new SparkMaxConfig();

  SparkRelativeEncoder mEncoder;

  public ArmIOSparkMax(int armcanid) {
    mSparkMax = new SparkMax(armcanid, MotorType.kBrushless);
    mEncoder = (SparkRelativeEncoder) mSparkMax.getEncoder();

    mConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.ArmConstants.CurrentLimit)
        .inverted(false);
    mConfig
        .encoder
        .positionConversionFactor(Constants.ArmConstants.PositionConversionFactor)
        .velocityConversionFactor(Constants.ArmConstants.VelocityConversionFactor);

    mSparkMax.configure(mConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.armPositionDegrees = mEncoder.getPosition();
    inputs.armVelocityDegreesPerSecond = mEncoder.getVelocity();

    mSparkMax.getAppliedOutput();
    mSparkMax.getBusVoltage();
    mSparkMax.getOutputCurrent();
    mSparkMax.getMotorTemperature();
  }

  public void armSpeedCommand(double speed) {
    mSparkMax.set(speed);
  }

  public void setVoltage(double voltage) {
    mSparkMax.setVoltage(voltage);
  }

  public void armStop() {
    mSparkMax.stopMotor();
    mSparkMax.disable();
  }
}
