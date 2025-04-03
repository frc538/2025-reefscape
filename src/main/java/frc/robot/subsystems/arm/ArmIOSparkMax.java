package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class ArmIOSparkMax implements ArmIO {
  private final SparkMax mSparkMax;
  private final SparkClosedLoopController mController;

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

    mConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.5, 0, 0)
        .outputRange(-1, 1);

    mSparkMax.configure(mConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mController = mSparkMax.getClosedLoopController();
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.armPositionDegrees = mEncoder.getPosition();
    inputs.armVelocityDegreesPerSecond = mEncoder.getVelocity();

    inputs.AppliedOutput = mSparkMax.getAppliedOutput();
    inputs.BusVoltage = mSparkMax.getBusVoltage();
    inputs.OutputCurrent = mSparkMax.getOutputCurrent();
    inputs.MotorTemperature = mSparkMax.getMotorTemperature();
  }

  public void armSpeedCommand(double speed, double ffvoltage) {
    mController.setReference(speed, ControlType.kVelocity,ClosedLoopSlot.kSlot0, ffvoltage);
  }

  public void setVoltage(double voltage) {
    mSparkMax.setVoltage(voltage);
  }

  public void armStop() {
    mSparkMax.stopMotor();
    mSparkMax.disable();
  }
}
