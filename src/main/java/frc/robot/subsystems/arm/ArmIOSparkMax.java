package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

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
        mConfig.encoder
                .positionConversionFactor(Constants.ArmConstants.PositionConversionFactor)
                .velocityConversionFactor(Constants.ArmConstants.VelocityConversionFactor);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.armPosition = mEncoder.getPosition();
        inputs.armVelocity = mEncoder.getVelocity();

        mSparkMax.getAppliedOutput();
        mSparkMax.getBusVoltage();
        mSparkMax.getOutputCurrent();
        mSparkMax.getMotorTemperature();
    }

    public void armSpeedCommand(double speed) {
        mSparkMax.set(speed);
    }

    public void armStop() {
        mSparkMax.stopMotor();
        mSparkMax.disable();
    }
}
