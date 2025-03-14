package frc.robot.subsystems.elevator;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorIOSparkMax implements ElevatorIO {
  SparkMax mLeft;
  SparkMax mRight;

  SparkRelativeEncoder mLeftEncoder;
  SparkRelativeEncoder mRightEncoder;

  SparkMaxConfig mLeftConfig = new SparkMaxConfig();
  SparkMaxConfig mRightConfig = new SparkMaxConfig();

  private final SparkClosedLoopController leftController;
  private final SparkClosedLoopController rightController;

  private final DigitalInput topLimit;
  private final DigitalInput bottomLimit;

  private boolean firstFrame = true;

  LoggedNetworkNumber kSnn = new LoggedNetworkNumber("/SmartDashboard/kS", 0.0);
  LoggedNetworkNumber kVnn = new LoggedNetworkNumber("/SmartDashboard/kV", 0.0);
  LoggedNetworkNumber kAnn = new LoggedNetworkNumber("/SmartDashboard/kA", 0.0);
  LoggedNetworkNumber maxVnn = new LoggedNetworkNumber("/SmartDashboard/maxV", 0.3);
  LoggedNetworkNumber maxAnn = new LoggedNetworkNumber("/SmartDashboard/maxA", 0.3);
  LoggedNetworkNumber Pnn = new LoggedNetworkNumber("/SmartDashboard/P", 0.0);
  LoggedNetworkNumber Inn = new LoggedNetworkNumber("/SmartDashboard/I", 0.0);
  LoggedNetworkNumber Dnn = new LoggedNetworkNumber("/SmartDashboard/D", 0.0);

  LoggedNetworkNumber gainIndexnn = new LoggedNetworkNumber("/SmartDashboard/Gain Index", 0);

  double kPLast = 0;
  double kILast = 0;
  double kDLast = 0;

  public ElevatorIOSparkMax(int leftId, int rightId, int upLimitChannel, int downLimitChannel) {
    mLeft = new SparkMax(leftId, MotorType.kBrushless);
    mRight = new SparkMax(rightId, MotorType.kBrushless);

    mLeftEncoder = (SparkRelativeEncoder) mLeft.getEncoder();
    mRightEncoder = (SparkRelativeEncoder) mRight.getEncoder();

    mRightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.ElevatorConstants.ElevatorCurrentLimit)
        .inverted(false);
    mRightConfig
        .encoder
        .positionConversionFactor(Constants.ElevatorConstants.ElevatorPositionConversionFactor)
        .velocityConversionFactor(Constants.ElevatorConstants.ElevatorVelocityConversionFactor);
    mRightConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.00, 0, 0)
        .outputRange(-1, 1);
    mRightConfig
        .closedLoop
        .maxMotion
        .allowedClosedLoopError(Constants.ElevatorConstants.allowedClosedLoopError)
        .maxAcceleration(Constants.ElevatorConstants.maxAcceleration)
        .maxVelocity(Constants.ElevatorConstants.maxVelocity);

    mLeftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.ElevatorConstants.ElevatorCurrentLimit)
        .inverted(true);
    mLeftConfig
        .encoder
        .positionConversionFactor(Constants.ElevatorConstants.ElevatorPositionConversionFactor)
        .velocityConversionFactor(Constants.ElevatorConstants.ElevatorVelocityConversionFactor);
    mLeftConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0, 0)
        .outputRange(-1, 1);
    mLeftConfig
        .closedLoop
        .maxMotion
        .allowedClosedLoopError(Constants.ElevatorConstants.allowedClosedLoopError)
        .maxAcceleration(Constants.ElevatorConstants.maxAcceleration)
        .maxVelocity(Constants.ElevatorConstants.maxVelocity);

    mLeft.configure(mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mRight.configure(mRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftController = mLeft.getClosedLoopController();
    rightController = mRight.getClosedLoopController();

    topLimit = new DigitalInput(upLimitChannel);
    bottomLimit = new DigitalInput(downLimitChannel);
  }

  // ¯\_(ツ)_/¯\\

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftBusVoltage = mLeft.getBusVoltage();
    inputs.leftCurrent = mLeft.getOutputCurrent();
    inputs.leftOutput = mLeft.getAppliedOutput();
    inputs.rightBusVoltage = mRight.getBusVoltage();
    inputs.rightCurrent = mRight.getOutputCurrent();
    inputs.rightOutput = mRight.getAppliedOutput();

    inputs.leftEncoderValue = mLeftEncoder.getPosition();
    inputs.rightEncoderValue = mRightEncoder.getPosition();

    inputs.atBottom = !bottomLimit.get();
    inputs.atTop = !topLimit.get();
  }

  public void setReference(double position, double ffCommand, double kP, double kI, double kD) {

    if (kPLast != kP || kILast != kI || kDLast != kD) {
      kPLast = kP;
      kILast = kI;
      kDLast = kD;
      mLeftConfig.closedLoop.pid(kP, kI, kD);
      mLeft.configure(
          mLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      mRightConfig.closedLoop.pid(kP, kI, kD);
      mRight.configure(
          mRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    if ((RobotState.isAutonomous() == true) || (RobotState.isTeleop())) {
      if (firstFrame == false) {
        leftController.setIAccum(0);
        rightController.setIAccum(0);
      }
      firstFrame = true;
    } else {
      firstFrame = false;
    }

    rightController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffCommand);
    leftController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffCommand);
  }

  @Override
  public void resetEncoders() {
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
  }

  @Override
  public void setEncoders(double position) {
    mLeftEncoder.setPosition(position);
    mRightEncoder.setPosition(position);
  }
}
