package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public class ElevatorIOSparkMax implements ElevatorIO {
    SparkMax mLeft;
    SparkMax mRight;

    SparkMaxConfig mLeftConfig;
    SparkMaxConfig mRightConfig;

    double mArbFF = 0.0;
    double mReferencePostition = 0.0;

    private final SparkClosedLoopController leftController;
    private final SparkClosedLoopController rightController;

    public ElevatorIOSparkMax(int leftId, int rightId) {
        mLeft = new SparkMax(leftId, MotorType.kBrushless);
        mRight = new SparkMax(rightId, MotorType.kBrushless);

        mRightConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Constants.ElevatorConstants.ElevatorCurrentLimit);
        mRightConfig.encoder
                .positionConversionFactor(Constants.ElevatorConstants.ElevatorPositionConversionFactor)
                .velocityConversionFactor(Constants.ElevatorConstants.ElevatorVelocityConversionFactor);
        mRightConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0, 0)
                .outputRange(-1, 1);
        mRightConfig.closedLoop.maxMotion
                .allowedClosedLoopError(Constants.ElevatorConstants.allowedClosedLoopError)
                .maxAcceleration(Constants.ElevatorConstants.maxAcceleration)
                .maxVelocity(Constants.ElevatorConstants.maxVelocity);

        mLeftConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Constants.ElevatorConstants.ElevatorCurrentLimit);
        mLeftConfig.encoder
                .positionConversionFactor(Constants.ElevatorConstants.ElevatorPositionConversionFactor)
                .velocityConversionFactor(Constants.ElevatorConstants.ElevatorVelocityConversionFactor);
        mLeftConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0, 0)
                .outputRange(-1, 1);
        mLeftConfig.closedLoop.maxMotion
                .allowedClosedLoopError(Constants.ElevatorConstants.allowedClosedLoopError)
                .maxAcceleration(Constants.ElevatorConstants.maxAcceleration)
                .maxVelocity(Constants.ElevatorConstants.maxVelocity);

        mLeft.configure(mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mRight.configure(mRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftController = mLeft.getClosedLoopController();
        rightController = mRight.getClosedLoopController();
    }

    //¯\_(ツ)_/¯\\

    public void updateInputs(ModuleIOInputs inputs) {
        
    }

    public void setReference(double position) {
        mReferencePostition = position;
        Logger.recordOutput("Sim/Commanded Position", position);
        System.out.println(position);
        leftController.setReference(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, mArbFF);
        rightController.setReference(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, mArbFF);
    }
    public void setArbFF(double arbFF) {
        mArbFF = arbFF;
        rightController.setReference(mReferencePostition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, mArbFF);
        leftController.setReference(mReferencePostition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, mArbFF);
    }
}
