package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.WristExtender.WristExtender;
import frc.robot.subsystems.WristExtender.WristExtenderIO;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

public class Elevator extends SubsystemBase {
  ElevatorIO io;

  private int positionTarget = 0;
  private int positionMax = 5;
  private int positionMin = 0;
  private double minGregHeight = 0.46;

  private double buttonPositionCommand = 0.0;
  private double PDotPositionCommand = 0.0;
  private double PDotRate = 0.0;
  private boolean UseButtonState = true;

  private final WristExtender wristPosition;

  private final double[] positionArray = {1, 2, 3};
  private final double[] gainArray = {1, 2, 3};

  Constraints profileConstraints;
  TrapezoidProfile commandProfile;
  ElevatorFeedforward m_feedforward;
  TrapezoidProfile.State mCurrentState;
  TrapezoidProfile.State mDesiredState;

  double mReferencePosition = 0.0;

  double[] positionThresholds = {1, 2};
  
  int gainIndex = 0;

  double[] kS = {0, 0, 0};
  double[] kG = {0.5, 0.65, 0.8};
  double[] kV = {3.5, 3.9, 4.5};
  double[] kA = {0, 0, 0};
  double[] maxV = {0.3, 0.3, 0.3};
  double[] maxA = {0.3, 0.3, 0.3};
  double[] kP = {0.5, 0.75, 0.75};
  double[] kI = {0.01, 0, 0};
  double[] kD = {0, 0, 0};

  // section heights
  // 0.420, 0.906, 1.313
  // 0.63, 1.36, 1.97
  // measured height, .628
  // .628 / .420 =

  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  LoggedNetworkNumber troughHeight = new LoggedNetworkNumber("/SmartDashboard/Trough Height", 0.46);
  LoggedNetworkNumber coralLowHeight = new LoggedNetworkNumber("/SmartDashboard/Coral Low Height", 0.81);
  LoggedNetworkNumber coralMedHeight = new LoggedNetworkNumber("/SmartDashboard/Coral Med Height", 1.21);
  LoggedNetworkNumber coralHighHeight = new LoggedNetworkNumber("/SmartDashboard/Coral High Height", 1.83);
  LoggedNetworkNumber bargeHeight = new LoggedNetworkNumber("/SmartDashboard/Barge Height", 1.9);

  LoggedNetworkNumber arbFF = new LoggedNetworkNumber("/SmartDashboard/Arbitrary FF Gain", 0.5);

  public Elevator(ElevatorIO IO, WristExtender WristExtender) {
    io = IO;
    wristPosition = WristExtender;

    profileConstraints = new Constraints(maxV[gainIndex], maxA[gainIndex]);
    commandProfile = new TrapezoidProfile(profileConstraints);

    m_feedforward =
        new ElevatorFeedforward(kS[gainIndex], kG[gainIndex], kV[gainIndex], kA[gainIndex]);
    mCurrentState = new TrapezoidProfile.State();
    mDesiredState = new TrapezoidProfile.State();
  }

  public Command PositionUp() {
    return runOnce(() -> doPositionUp());
  }

  private void doPositionUp() {
    System.out.println("PositionUp");
    if (positionTarget < positionMax) {
      positionTarget = positionTarget + 1;
    }
    goToPosition();
  }

  public Command PositionDown() {
    return runOnce(() -> doPositionDown());
  }

  private void doPositionDown() {
    System.out.println("PositionDown");
    if (positionTarget > positionMin) {
      positionTarget = positionTarget - 1;
    }
    goToPosition();
  }

  private void goToPosition() {
    switch (positionTarget) {
      case 0:
        PositionCommand(0);
        break;
      case 1:
        PositionCommand(troughHeight.get());
        break;
      case 2:
        PositionCommand(coralLowHeight.get());
        break;
      case 3:
        PositionCommand(coralMedHeight.get());
        break;
      case 4:
        PositionCommand(coralHighHeight.get());
        break;
      case 5:
        PositionCommand(bargeHeight.get());
        break;
      default:
        break;
    }
  }

  public Command Bottom() {
    return runOnce(() -> PositionCommand(0));
  }

  public Command Trough() {
    return runOnce(() -> PositionCommand(troughHeight.get()));
  }

  public Command CoralLow() {
    return runOnce(() -> PositionCommand(coralLowHeight.get()));
  }

  public Command CoralMed() {
    return runOnce(() -> PositionCommand(coralMedHeight.get()));
  }

  public Command CoralHigh() {
    return runOnce(() -> PositionCommand(coralHighHeight.get()));
  }

  public Command Barge() {
    return runOnce(() -> PositionCommand(bargeHeight.get()));
  }

  private void PositionCommand(double position) {
    if (position < minGregHeight && wristPosition.isGregoryDown() == true)  {
      position = minGregHeight;
    } 
      setReference(position);
      buttonPositionCommand = position;
      PDotPositionCommand = position;
      UseButtonState = true;
  }

  public Command PDotCommand(double rate) {
    return run(() -> doRate(rate));
  }

  private void doRate(double rate) {
    if (rate != 0) {
      PDotRate = rate;
      UseButtonState = false;
      PDotPositionCommand = PDotPositionCommand + PDotRate;
      if (PDotPositionCommand < minGregHeight && wristPosition.isGregoryDown() == true) {
        PDotPositionCommand = minGregHeight;
      }
      if (PDotPositionCommand < 0) {
        PDotPositionCommand = 0;
      }
      setReference(PDotPositionCommand);
    }
  }

  private void setReference(double position) {
    mReferencePosition = position;
    mDesiredState = new TrapezoidProfile.State(mReferencePosition, 0);
    Logger.recordOutput("Elevator/Commanded Position", position);
  }

  private void commandMotors(){
     double ffCommand = 0;
    TrapezoidProfile.State state_step = new TrapezoidProfile.State();
    double averagePosition = (inputs.leftEncoderValue + inputs.rightEncoderValue) / 2;
    if (averagePosition > positionThresholds[1]) {
      gainIndex = 2;
    } else if (averagePosition > positionThresholds[0]) {
      gainIndex = 1;
    } else {
      gainIndex = 0;
    }

    // kS[(int) gainIndexnn.get()] = kSnn.get();
    // kV[(int) gainIndexnn.get()] = kVnn.get();
    // kA[(int) gainIndexnn.get()] = kAnn.get();

    // kP[(int) gainIndexnn.get()] = Pnn.get();
    // kI[(int) gainIndexnn.get()] = Inn.get();
    // kD[(int) gainIndexnn.get()] = Dnn.get();

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

    m_feedforward =
        new ElevatorFeedforward(kS[gainIndex], kG[gainIndex], kV[gainIndex], kA[gainIndex]);

    if ((RobotState.isAutonomous() == true) || (RobotState.isTeleop())){
        /* Do the command processing */
      state_step = commandProfile.calculate(0.02, mCurrentState, mDesiredState);
      mCurrentState = state_step;
      ffCommand =
          m_feedforward.calculateWithVelocities(mCurrentState.velocity, state_step.velocity);
      io.setReference(state_step.position, ffCommand, kP[gainIndex], kI[gainIndex], kD[gainIndex]);

    } 
    else {
      // do stuff when disabled
      // init reference position to where it thinks it is as average.
      mReferencePosition = (inputs.rightEncoderValue + inputs.leftEncoderValue) / 2;
      PDotPositionCommand = mReferencePosition;
      buttonPositionCommand = mReferencePosition;
      mDesiredState = new TrapezoidProfile.State(mReferencePosition, 0);
      mCurrentState = mDesiredState;
    }
    Logger.recordOutput("Elevator/Feed Forward Command", ffCommand);
    Logger.recordOutput("Elevator/Motor Position Command", mReferencePosition);
    Logger.recordOutput("Elevator/Profile/Position", state_step.position);
    Logger.recordOutput("Elevator/Profile/Velocity", state_step.velocity);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Trough Height", troughHeight.get());
    if (UseButtonState) {
      Logger.recordOutput("Elevator/Command Source", "Button");
    } else {
      Logger.recordOutput("Elevator/Command Source", "Stick");
    }

    io.setArbFF(arbFF.get());

    if (inputs.atBottom) {
      io.resetEncoders();
    }

    commandMotors();

    Logger.recordOutput("Elevator/Button Command", buttonPositionCommand);
    Logger.recordOutput("Elevator/PDot Command", PDotPositionCommand);
    Logger.recordOutput("Elevator/Position Target", positionTarget);
  }
}
