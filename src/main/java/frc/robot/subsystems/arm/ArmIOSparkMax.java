package frc.robot.subsystems.arm;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class ArmIOSparkMax implements ArmIO {
  private final SparkMax mSparkMax;
  private final SparkClosedLoopController mController;

  private double kp = 0.0006;
  private double ki = 0;
  private double kd = 0;

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
        .positionConversionFactor(
            Constants.ArmConstants.PositionConversionFactor) // Degrees of arm motion
        .velocityConversionFactor(
            Constants.ArmConstants.VelocityConversionFactor); // Arm degrees per second

    mConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kp, ki, kd)
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

  @Override
  public void setReference(double speed, double ffvoltage) {
    mController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffvoltage);
  }

  public void setVoltage(double voltage) {
    mSparkMax.setVoltage(voltage);
  }

  public void armStop() {
    mSparkMax.stopMotor();
    mSparkMax.disable();
  }
}
