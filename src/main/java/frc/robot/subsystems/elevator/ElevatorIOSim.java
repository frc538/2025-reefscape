package frc.robot.subsystems.elevator;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorIOSim implements ElevatorIO {
  // Use only the left side, leaving the DCMotor model handle twice the effort
  SparkMax mLeft;
  SparkRelativeEncoder mLeftEncoder;
  SparkMaxConfig mLeftConfig = new SparkMaxConfig();
  TrapezoidProfile commandProfile;
  Constraints profileConstraints;
  TrapezoidProfile.State mCurrentState;
  TrapezoidProfile.State mDesiredState;
  ElevatorFeedforward m_feedforward;

  private boolean firstFrame = true;

  LoggedNetworkNumber kSnn = new LoggedNetworkNumber("/SmartDashboard/kS", 0);
  LoggedNetworkNumber kVnn = new LoggedNetworkNumber("/SmartDashboard/kV", 3.3);
  LoggedNetworkNumber kAnn = new LoggedNetworkNumber("/SmartDashboard/kA", 0.5);
  LoggedNetworkNumber maxVnn = new LoggedNetworkNumber("/SmartDashboard/maxV", 0.3);
  LoggedNetworkNumber maxAnn = new LoggedNetworkNumber("/SmartDashboard/maxA", 0.3);
  LoggedNetworkNumber Pnn = new LoggedNetworkNumber("/SmartDashboard/P", 0.0);
  LoggedNetworkNumber Inn = new LoggedNetworkNumber("/SmartDashboard/I", 0);
  LoggedNetworkNumber Dnn = new LoggedNetworkNumber("/SmartDashboard/D", 0);

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

  double kPLast = 0;
  double kILast = 0;
  double kDLast = 0;

  private final SparkClosedLoopController leftController;

  /*
   * Gear 1 - 2106 MAXPlanetary Universal Input Stage
   * Gear 2 - 2129 MAXPlanetary 9:1 Cartridge
   * Gear 3 - 2104 MAXPlanetary 1/2" Hex Socket Output
   * AM4779 - 26T #25 SERIES - pitch diameter 2.074
   *
   * Idler sprockets - 19T - 41mm OD
   *
   *
   */

  // This gearbox represents a gearbox containing 2 NEOs.
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);
  private final double kElevatorGearing = 9.0;
  private final double kCarriageMass = 10.0;
  private final double kElevatorDrumRadius = Units.inchesToMeters(2.074);
  private final double kMinElevatorHeightMeters = 0.0;
  private final double kMaxElevatorHeightMeters = Units.feetToMeters(6);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          kElevatorGearing,
          kCarriageMass,
          kElevatorDrumRadius,
          kMinElevatorHeightMeters,
          kMaxElevatorHeightMeters,
          true,
          0,
          0, // 0.01,
          0.0);

  // create the Spark MAX sim object
  SparkMaxSim mSimLeft;
  SparkRelativeEncoderSim mLeftEncoderSim;

  public ElevatorIOSim(int leftId) {
    mLeft = new SparkMax(leftId, MotorType.kBrushless);
    mLeftEncoder = (SparkRelativeEncoder) mLeft.getEncoder();

    mSimLeft = new SparkMaxSim(mLeft, m_elevatorGearbox);
    mLeftEncoderSim = mSimLeft.getRelativeEncoderSim();

    mLeftConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(Constants.ElevatorConstants.ElevatorCurrentLimit);
    mLeftConfig
        .encoder
        .positionConversionFactor(Constants.ElevatorConstants.ElevatorPositionConversionFactor)
        .velocityConversionFactor(Constants.ElevatorConstants.ElevatorVelocityConversionFactor);
    mLeftConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0, 0, 0)
        .outputRange(-1, 1);

    mLeft.configure(mLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftController = mLeft.getClosedLoopController();

    profileConstraints = new Constraints(maxV, maxA);
    commandProfile = new TrapezoidProfile(profileConstraints);
    m_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    mCurrentState = new TrapezoidProfile.State();
    setReference(0.0);
  }

  // ¯\_(ツ)_/¯\\

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftBusVoltage = mSimLeft.getAppliedOutput() * RoboRioSim.getVInVoltage();
    inputs.leftCurrent = mSimLeft.getMotorCurrent();
    inputs.leftOutput = mSimLeft.getAppliedOutput();
    // inputs.height = m_elevatorSim.getPositionMeters();

    inputs.leftEncoderValue = mLeftEncoder.getPosition();
  }

  public void setReference(double position) {
    mReferencePosition = position;
    mDesiredState = new TrapezoidProfile.State(mReferencePosition, 0);
    Logger.recordOutput("Sim/Commanded Position", position);
    System.out.println(position);
  }

  public void setArbFF(double arbFF) {
    mArbFF = arbFF;
    kG = arbFF;
    m_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  }

  public void setReference(double position, double ffCommand, double kP, double kI, double kD) {
    if (kPLast != kP || kILast != kI || kDLast != kD) {
      kPLast = kP;
      kILast = kI;
      kDLast = kD;
      mLeftConfig.closedLoop.pid(kP, kI, kD);
      mLeft.configure(
          mLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    if ((RobotState.isAutonomous() == true) || (RobotState.isTeleop())) {
      if (firstFrame == false) {
        leftController.setIAccum(0);
      }
      firstFrame = true;
    } else {
      firstFrame = false;
    }

    leftController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffCommand);
    /* Run the simulation components */
    m_elevatorSim.setInput(mSimLeft.getAppliedOutput() * RoboRioSim.getVInVoltage());
    m_elevatorSim.update(0.02);

    Logger.recordOutput(
        "Elevator/Sim/Elevator Velocity", m_elevatorSim.getVelocityMetersPerSecond());
    Logger.recordOutput("Elevator/Sim/Elevator Position", m_elevatorSim.getPositionMeters());

    /*
     * Ve = elevator speed m/s
     * ElevatorPositionConversionFactor = m/rotation conversion
     * Ve / ElevatorPositionConversionFactor (rotations / s)
     * Ve / ElevatorPositionConversionFactor / 60s/min (rotations/min)
     */
    mSimLeft.iterate(
        m_elevatorSim.getVelocityMetersPerSecond() * 60.0,
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        0.02);

    Logger.recordOutput("Elevator/Sim/Motor Position", mSimLeft.getPosition());
    Logger.recordOutput("Elevator/Sim/Motor Velocity", mSimLeft.getVelocity());

    Logger.recordOutput("Elevator/Sim/Encoder Position", mLeftEncoderSim.getPosition());

    // mLeftEncoderSim.iterate(mLeftEncoderSim.getVelocity() *
    // Constants.ElevatorConstants.ElevatorPositionConversionFactor,0.02);

    // SimBattery estimates loaded battery voltages
    // This should include all motors being simulated
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(mSimLeft.getMotorCurrent()));
  }

  public void setEncoders(double position) {
    mLeftEncoder.setPosition(position);
  }
}
