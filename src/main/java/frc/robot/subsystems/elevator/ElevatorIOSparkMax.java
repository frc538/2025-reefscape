package frc.robot.subsystems.elevator;

import java.lang.reflect.Array;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.RobotState;

public class ElevatorIOSparkMax implements ElevatorIO {
        SparkMax mLeft;
        SparkMax mRight;

        SparkRelativeEncoder mLeftEncoder;
        SparkRelativeEncoder mRightEncoder;

        SparkMaxConfig mLeftConfig = new SparkMaxConfig();
        SparkMaxConfig mRightConfig = new SparkMaxConfig();

        private final SparkClosedLoopController leftController;
        private final SparkClosedLoopController rightController;

        private final double[] positionArray = { 1, 2, 3 };
        private final double[] gainArray = { 1, 2, 3 };

        private boolean firstFrame;

        private final DigitalInput topLimit;
        private final DigitalInput bottomLimit;

        Constraints profileConstraints;
        TrapezoidProfile commandProfile;
        ElevatorFeedforward m_feedforward;
        TrapezoidProfile.State mCurrentState;
        TrapezoidProfile.State mDesiredState;

        LoggedNetworkNumber kSnn = new LoggedNetworkNumber("/SmartDashboard/kS", 0.0);
        LoggedNetworkNumber kVnn = new LoggedNetworkNumber("/SmartDashboard/kV", 0.0);
        LoggedNetworkNumber kAnn = new LoggedNetworkNumber("/SmartDashboard/kA", 0.0);
        LoggedNetworkNumber maxVnn = new LoggedNetworkNumber("/SmartDashboard/maxV", 0.3);
        LoggedNetworkNumber maxAnn = new LoggedNetworkNumber("/SmartDashboard/maxA", 0.3);
        LoggedNetworkNumber Pnn = new LoggedNetworkNumber("/SmartDashboard/P", 0.0);
        LoggedNetworkNumber Inn = new LoggedNetworkNumber("/SmartDashboard/I", 0.0);
        LoggedNetworkNumber Dnn = new LoggedNetworkNumber("/SmartDashboard/D", 0.0);

        LoggedNetworkNumber gainIndexnn = new LoggedNetworkNumber("/SmartDashboard/Gain Index", 0);

        double mReferencePosition = 0.0;

        double[] kS = { 0, 0, 0 };
        double[] kG = { 0, 0, 0 };
        double[] kV = { 0, 0, 0 };
        double[] kA = { 0, 0, 0 };
        double[] maxV = { 0.3, 0.3, 0.3 };
        double[] maxA = { 0.3, 0.3, 0.3 };
        double[] kP = { 0.001, 0.001, 0.001 };
        double[] kI = { 0, 0, 0 };
        double[] kD = { 0, 0, 0 };
        int gainIndex = 0;
        double[] positionThresholds = { 1, 2 };
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
                mRightConfig.encoder
                                .positionConversionFactor(Constants.ElevatorConstants.ElevatorPositionConversionFactor)
                                .velocityConversionFactor(Constants.ElevatorConstants.ElevatorVelocityConversionFactor);
                mRightConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(0.00, 0, 0)
                                .outputRange(-1, 1);
                mRightConfig.closedLoop.maxMotion
                                .allowedClosedLoopError(Constants.ElevatorConstants.allowedClosedLoopError)
                                .maxAcceleration(Constants.ElevatorConstants.maxAcceleration)
                                .maxVelocity(Constants.ElevatorConstants.maxVelocity);

                mLeftConfig
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(Constants.ElevatorConstants.ElevatorCurrentLimit)
                                .inverted(true);
                mLeftConfig.encoder
                                .positionConversionFactor(Constants.ElevatorConstants.ElevatorPositionConversionFactor)
                                .velocityConversionFactor(Constants.ElevatorConstants.ElevatorVelocityConversionFactor);
                mLeftConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(0.0, 0, 0)
                                .outputRange(-1, 1);
                mLeftConfig.closedLoop.maxMotion
                                .allowedClosedLoopError(Constants.ElevatorConstants.allowedClosedLoopError)
                                .maxAcceleration(Constants.ElevatorConstants.maxAcceleration)
                                .maxVelocity(Constants.ElevatorConstants.maxVelocity);

                mLeft.configure(mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                mRight.configure(mRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                leftController = mLeft.getClosedLoopController();
                rightController = mRight.getClosedLoopController();

                profileConstraints = new Constraints(maxV[gainIndex], maxA[gainIndex]);
                commandProfile = new TrapezoidProfile(profileConstraints);

                m_feedforward = new ElevatorFeedforward(kS[gainIndex], kG[gainIndex], kV[gainIndex], kA[gainIndex]);
                mCurrentState = new TrapezoidProfile.State();
                mDesiredState = new TrapezoidProfile.State();

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

        public void setReference(double position) {
                mReferencePosition = position;
                mDesiredState = new TrapezoidProfile.State(mReferencePosition, 0);
                Logger.recordOutput("Elevator/Commanded Position", position);
        }

        public void setArbFF(double arbFF) {
                kG[gainIndex] = arbFF;
        }

        public void commandMotor() {
                double ffCommand = 0;
                TrapezoidProfile.State state_step = new TrapezoidProfile.State();
                double averagePosition = (mLeftEncoder.getPosition() + mRightEncoder.getPosition()) / 2;
                if (averagePosition > positionThresholds[1]) {
                        gainIndex = 2;
                } else if (averagePosition > positionThresholds[0]) {
                        gainIndex = 1;
                } else {
                        gainIndex = 0;
                }

                kS[(int) gainIndexnn.get()] = kSnn.get();
                kV[(int) gainIndexnn.get()] = kVnn.get();
                kA[(int) gainIndexnn.get()] = kAnn.get();

                kP[(int) gainIndexnn.get()] = Pnn.get();
                kI[(int) gainIndexnn.get()] = Inn.get();
                kD[(int) gainIndexnn.get()] = Dnn.get();

                // Always update gains when read in from the network numbers into the gainIndex
                // value
                // Change the code to check if the configured parameter is different from the
                // current gain for that parameter at gainIndex
                // If it is different, then either the NN number was reprogrammed, or the
                // elevator has changed gain region
                // So reprogram that piece

                if (profileConstraints.maxAcceleration != maxA[gainIndex]
                                || profileConstraints.maxVelocity != maxV[gainIndex]) {
                        profileConstraints = new Constraints(maxV[gainIndex], maxA[gainIndex]);
                        commandProfile = new TrapezoidProfile(profileConstraints);
                }

                m_feedforward = new ElevatorFeedforward(kS[gainIndex], kG[gainIndex], kV[gainIndex], kA[gainIndex]);

                if (kPLast != kP[gainIndex] || kILast != kI[gainIndex]
                                || kDLast != kD[ gainIndex]) {
                        kPLast = kP[gainIndex];
                        kILast = kI[gainIndex];
                        kDLast = kD[gainIndex];
                        mLeftConfig.closedLoop.pid(kP[gainIndex], kI[gainIndex], kD[gainIndex]);
                        mLeft.configure(mLeftConfig, ResetMode.kNoResetSafeParameters,
                                        PersistMode.kNoPersistParameters);

                        mRightConfig.closedLoop.pid(kP[gainIndex], kI[gainIndex], kD[gainIndex]);
                        mRight.configure(mRightConfig, ResetMode.kNoResetSafeParameters,
                                        PersistMode.kNoPersistParameters);
                }

                if ((RobotState.isAutonomous() == true) || (RobotState.isTeleop())) {
                        if (firstFrame == false) {
                                leftController.setIAccum(0);
                                rightController.setIAccum(0);
                        }
                        firstFrame = true;

                        /* Do the command processing */
                        state_step = commandProfile.calculate(0.02, mCurrentState,
                                        mDesiredState);
                        mCurrentState = state_step;
                        ffCommand = m_feedforward.calculateWithVelocities(mCurrentState.velocity,
                                        state_step.velocity);
                        leftController.setReference(state_step.position, ControlType.kPosition,
                                        ClosedLoopSlot.kSlot0,
                                        ffCommand);
                        rightController.setReference(state_step.position, ControlType.kPosition,
                                        ClosedLoopSlot.kSlot0,
                                        ffCommand);
                } else {
                        firstFrame = false;
                        // do stuff when disabled
                        // init reference position to where it thinks it is as average.
                        mReferencePosition = (mRightEncoder.getPosition() + mLeftEncoder.getPosition()) / 2;
                        mDesiredState = new TrapezoidProfile.State(mReferencePosition, 0);
                        mCurrentState = mDesiredState;
                }

                Logger.recordOutput("Elevator/Feed Forward Command", ffCommand);
                Logger.recordOutput("Elevator/Motor Position Command", mReferencePosition);
                Logger.recordOutput("Elevator/Profile/Position", state_step.position);
                Logger.recordOutput("Elevator/Profile/Velocity", state_step.velocity);
        }

        @Override
        public void resetEncoders() {
                mLeftEncoder.setPosition(0);
                mRightEncoder.setPosition(0);
        }
}
