package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Elevator extends SubsystemBase {
  ElevatorIO io;

  private int positionTarget = 0;
  private int positionMax = 5;
  private int positionMin = 0;
  private double calibratedGregStartingHeight = 0.516;
  private double lowestObservedPosition = calibratedGregStartingHeight;
  private double highestObservedPosition = calibratedGregStartingHeight;
  private boolean bottomSwitchHit = false;
  private boolean topSwitchHit = false;

  private double buttonPositionCommand = 0.0;
  private double PDotPositionCommand = 0.0;
  private double PDotRate = 0.0;
  private boolean UseButtonState = true;
  private double positionReset;

  Constraints profileConstraints;
  TrapezoidProfile commandProfile;
  ElevatorFeedforward m_feedforward;
  TrapezoidProfile.State mCurrentState;
  TrapezoidProfile.State mDesiredState;

  double mReferencePosition = 0.0;

  double[] positionThresholds = {0.55, 1.28};

  int gainIndex = 0;

  double[] kS = {0, 0, 0.5};
  double[] kG = {0.9, 1.1, 1.25};
  double[] kV = {2.5, 2.5, 2.5};
  double[] kA = {1, 1, 1};
  double[] maxV = {0.3, 0.3, 0.3};
  double[] maxA = {0.3, 0.3, 0.3};
  double[] kP = {.75, .75, .75};
  double[] kI = {0, 0, 0};
  double[] kD = {0, 0, 0};

  // section heights
  // 0.420, 0.906, 1.313
  // 0.63, 1.36, 1.97
  // measured height, .628
  // .628 / .420 =

  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  LoggedNetworkNumber stowedHeight = new LoggedNetworkNumber("/SmartDashboard/Stowed Height", 0.516);
  LoggedNetworkNumber algaeLowHeight =
      new LoggedNetworkNumber("/SmartDashboard/Algae Low Height", 0.81);
  LoggedNetworkNumber algaeHighHeight =
      new LoggedNetworkNumber("/SmartDashboard/Algae High Height", 1.2);
  LoggedNetworkNumber bargeHeight = new LoggedNetworkNumber("/SmartDashboard/Barge Height", 1.9);

  LoggedNetworkNumber arbFF = new LoggedNetworkNumber("/SmartDashboard/Arbitrary FF Gain", 0.5);

  public Elevator(ElevatorIO IO) {
    io = IO;

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
        PositionCommand(stowedHeight.get());
        break;
      case 1:
        PositionCommand(algaeLowHeight.get());
        break;
      case 2:
        PositionCommand(algaeHighHeight.get());
        break;
      case 3:
        PositionCommand(bargeHeight.get());
        break;
      default:
        break;
    }
  }

  public Command Stowed() {
    return runOnce(() -> PositionCommand(stowedHeight.get()));
  }

  public Command algaeLow() {
    return runOnce(() -> PositionCommand(algaeLowHeight.get()));
  }

  public Command algaeMed() {
    return runOnce(() -> PositionCommand(algaeHighHeight.get()));
  }

  public Command Barge() {
    return runOnce(() -> PositionCommand(bargeHeight.get()));
  }

  private void PositionCommand(double position) {
    if (bottomSwitchHit == false && position < lowestObservedPosition) {
      position = lowestObservedPosition;
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
      // if (PDotPositionCommand < minGregHeight && wristPosition.isGregoryDown() == true) {
      //  PDotPositionCommand = minGregHeight;
      // }
      if (PDotPositionCommand < 0 && bottomSwitchHit == true) {
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

  private void commandMotors() {
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

    if (RobotState.isEnabled() == true) {
      /* Do the command processing */
      state_step = commandProfile.calculate(0.02, mCurrentState, mDesiredState);
      mCurrentState = state_step;
      ffCommand =
          m_feedforward.calculateWithVelocities(mCurrentState.velocity, state_step.velocity);
    } else {
      // do stuff when disabled
      // init reference position to where it thinks it is as average.
      // TODO ************* use average of both encoders on real robot **************
      mReferencePosition = calibratedGregStartingHeight;
      PDotPositionCommand = mReferencePosition;
      buttonPositionCommand = mReferencePosition;
      mDesiredState = new TrapezoidProfile.State(mReferencePosition, 0);
      mCurrentState = mDesiredState;
      lowestObservedPosition = mReferencePosition;
    }
    io.setReference(mCurrentState.position, ffCommand, kP[gainIndex], kI[gainIndex], kD[gainIndex]);
    Logger.recordOutput("Elevator/Feed Forward Command", ffCommand);
    Logger.recordOutput("Elevator/Motor Position Command", mReferencePosition);
    Logger.recordOutput("Elevator/Profile/Position", mCurrentState.position);
    Logger.recordOutput("Elevator/Profile/Velocity", mCurrentState.velocity);
    Logger.recordOutput("Elevator/Lowest Observed Position", lowestObservedPosition);
    Logger.recordOutput("Elevator/Has Bottom Been Hit", bottomSwitchHit);
    Logger.recordOutput("Elevator/gainIndex",gainIndex);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    if (UseButtonState) {
      Logger.recordOutput("Elevator/Command Source", "Button");
    } else {
      Logger.recordOutput("Elevator/Command Source", "Stick");
    }

    if (inputs.atBottom) {
      io.resetEncoders();
      lowestObservedPosition = 0;
      bottomSwitchHit = true;
    } else {
      if (lowestObservedPosition > ((inputs.leftEncoderValue + inputs.rightEncoderValue) / 2)) {
        lowestObservedPosition = ((inputs.leftEncoderValue + inputs.rightEncoderValue) / 2);
      }
    }

    commandMotors();

    Logger.recordOutput("Elevator/Button Position Command", buttonPositionCommand);
    Logger.recordOutput("Elevator/PDot Command", PDotPositionCommand);
    Logger.recordOutput("Elevator/Position Target", positionTarget);
    Logger.recordOutput("Elevator/Section", gainIndex);
  }
}
