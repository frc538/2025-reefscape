package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
        SparkMax mLeft;
        SparkMax mRight;

        SparkMaxConfig mLeftConfig;
        SparkMaxConfig mRightConfig;

        private final SparkClosedLoopController leftController;
        private final SparkClosedLoopController rightController;

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

        double mReferencePosition = 0.0;
        double mArbFF = 0.0;
        double kS = 0.0;
        double kG = 0.0;
        double kV = 0.0;
        double kA = 0.0;
        double maxV = 0.3;
        double maxA = 0.3;
        double kP = 0.0;
        double kI = 0.0;
        double kD = 0.0;

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

        // ¯\_(ツ)_/¯\\

        public void updateInputs(ElevatorIOInputs inputs) {
                inputs.leftBusVoltage = mLeft.getBusVoltage();
                inputs.leftCurrent = mLeft.getOutputCurrent();
                inputs.leftOutput = mLeft.getAppliedOutput();
                inputs.rightBusVoltage = mRight.getBusVoltage();
                inputs.rightCurrent = mRight.getOutputCurrent();
                inputs.rightOutput = mRight.getAppliedOutput();
        }

        public void setReference(double position) {
                leftController.setReference(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0,
                                Constants.ElevatorConstants.arbitraryFeedForward);
                rightController.setReference(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0,
                                Constants.ElevatorConstants.arbitraryFeedForward);
        }

        public void commandMotor() {
                kS = kSnn.get();
                kV = kVnn.get();
                kA = kAnn.get();
                if (maxV != maxVnn.get() || maxA != maxAnn.get()) {
                        maxV = maxVnn.get();
                        maxA = maxAnn.get();
                        profileConstraints = new Constraints(maxV, maxA);
                        commandProfile = new TrapezoidProfile(profileConstraints);
                }

                m_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

                if (Pnn.get() != kP || Inn.get() != kI || Dnn.get() != kD) {
                        kP = Pnn.get();
                        kI = Inn.get();
                        kD = Dnn.get();
                        mLeftConfig.closedLoop.pid(kP, kI, kD);
                        mLeft.configure(mLeftConfig, ResetMode.kNoResetSafeParameters,
                                        PersistMode.kNoPersistParameters);
                }

                /* Do the command processing */
                TrapezoidProfile.State state_step = commandProfile.calculate(0.02, mCurrentState, mDesiredState);
                mCurrentState = state_step;
                double ffCommand = m_feedforward.calculateWithVelocities(mCurrentState.velocity, state_step.velocity);
                leftController.setReference(mReferencePosition, ControlType.kPosition, ClosedLoopSlot.kSlot0,
                                ffCommand);

                Logger.recordOutput("Elevator/Feed Forward Command", ffCommand);
                Logger.recordOutput("Elevator/Profile/Position", state_step.position);
                Logger.recordOutput("Elevator/Profile/Velocity", state_step.velocity);
        }
}
