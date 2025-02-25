package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
    SparkMax mLeft;
    SparkMax mRight;

    SparkMaxConfig mLeftConfig;
    SparkMaxConfig mRightConfig;

    public ElevatorIOSparkMax(int leftId, int rightId) {
        mLeft = new SparkMax(leftId, MotorType.kBrushless);
        mRight = new SparkMax(rightId, MotorType.kBrushless);

        mRightConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Constants.ModuleConstants.DriveCurrentLimit);
        mRightConfig.encoder
                .positionConversionFactor(Constants.ModuleConstants.RightPositionConversionFactor)
                .velocityConversionFactor(Constants.ModuleConstants.RightVelocityConversionFactor);
        mRightConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0, 0)
                .velocityFF(1 / Constants.ModuleConstants.RightMotorFreeSpeedRevsPerSecond)
                .outputRange(-1, 1);

        mLeftConfig
                .idleMode(IdleMode.kBrake)
                .follow(mRight);
        mLeftConfig.encoder
                .positionConversionFactor(Constants.ModuleConstants.LeftPositionConversionFactor)
                .velocityConversionFactor(Constants.ModuleConstants.LeftVelocityConversionFactor);
        mLeftConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0, 0)
                .velocityFF(1 / Constants.ModuleConstants.LeftMotorFreeSpeedRevsPerSecond)
                .outputRange(-1, 1);
    }
}
