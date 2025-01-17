// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModuleIOSparkmax implements SwerveModuleIO {

  private final SparkMax driveMotor;
  private final SparkMax turnMotor;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;
  private final SparkClosedLoopController drivePID;
  private final SparkClosedLoopController turnPID;

  private final SparkMaxConfig driveConfig = new SparkMaxConfig();
  private final SparkMaxConfig turnConfig = new SparkMaxConfig();

  private double mAngularOffset = 0;
  private SwerveModuleState mDesiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModuleIOSparkmax(int driveId, int turnId, double offset) {
    driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    turnMotor = new SparkMax(turnId, MotorType.kBrushless);

    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.ModuleConstants.DriveCurrentLimit);

    driveConfig
        .encoder
        .positionConversionFactor(Constants.ModuleConstants.DrivePositionConversionFactor)
        .velocityConversionFactor(Constants.ModuleConstants.DriveVelocityConversionFactor);

    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.04, 0, 0)
        .velocityFF(1 / Constants.ModuleConstants.DriveMotorFreeSpeedRevsPerSecond)
        .outputRange(-1, 1);

    // driveConfig.inverted(false).idleMode(IdleMode.kBrake);
    // driveConfig
    //     .encoder
    //     .positionConversionFactor(Constants.ModuleConstants.DrivePositionConversionFactor)
    //     .velocityConversionFactor(Constants.ModuleConstants.DriveVelocityConversionFactor);
    // driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.4, 0.0, 0.0);

    // driveConfig.smartCurrentLimit(Constants.ModuleConstants.DriveCurrentLimit);

    // driveMotor.configure(
    //     driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.ModuleConstants.TurnCurrentLimit);

    turnConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor(Constants.ModuleConstants.TurnPositionConversionFactor)
        .velocityConversionFactor(Constants.ModuleConstants.TurnVelocityConversionFactor);

    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(1, 0, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 2 * Math.PI);

    // turnConfig.inverted(true).idleMode(IdleMode.kBrake);
    // turnConfig
    //     .encoder
    //     .positionConversionFactor(Constants.ModuleConstants.TurnPositionConversionFactor)
    //     .velocityConversionFactor(Constants.ModuleConstants.TurnVelocityConversionFactor);
    // turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(1.0, 0.0, 0.0);

    // turnConfig.smartCurrentLimit(Constants.ModuleConstants.TurnCurrentLimit);

    // turnMotor.configure(
    //     driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    drivePID = driveMotor.getClosedLoopController();
    turnPID = turnMotor.getClosedLoopController();

    mAngularOffset = offset;

    mDesiredState.angle = new Rotation2d(turnEncoder.getPosition());

    driveEncoder.setPosition(0);
  }

  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.state =
        new SwerveModuleState(
            driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition() - mAngularOffset));

    inputs.position =
        new SwerveModulePosition(
            driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition() - mAngularOffset));
  }

  public void reset() {
    driveEncoder.setPosition(0);
  }

  public void setmDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedState = new SwerveModuleState();
    correctedState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(mAngularOffset));

    SwerveModuleState optimizedState =
        SwerveModuleState.optimize(correctedState, new Rotation2d(turnEncoder.getPosition()));

    drivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
    turnPID.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);

    mDesiredState = desiredState;
  }
}
