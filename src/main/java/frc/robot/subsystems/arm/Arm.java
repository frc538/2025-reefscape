package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double mSpeed = 0.0;

  Constraints profileConstraints;
  TrapezoidProfile commandProfile;
  ArmFeedforward m_feedforward;
  TrapezoidProfile.State mCurrentState;
  TrapezoidProfile.State mDesiredState;

  double maxA = 90;
  double maxV = 45;
  double kS = 0;
  double kG = 0;//simple feed forward control
  double kV = 0;

  boolean simpleControl = true;//position control loop.
  double RateCommand = 0;

  double PDotPositionCommand = 0;
  double PDotRate = 0;
  boolean UseButtonState = false;
  double mReferencePosition = 0;

  public Arm(ArmIO IO) {
    io = IO;

    profileConstraints = new Constraints(maxV, maxA);
    commandProfile = new TrapezoidProfile(profileConstraints);

    m_feedforward = new ArmFeedforward(kS, kG, kV);
    mCurrentState = new TrapezoidProfile.State();
    mDesiredState = new TrapezoidProfile.State();
  }

  public Command MoveArm(DoubleSupplier speedSupplier) {
    return run(
        () -> {
          mSpeed = speedSupplier.getAsDouble();
          mSpeed = MathUtil.applyDeadband(mSpeed, 0.1);
          io.armSpeedCommand(mSpeed);
        });
  }

  public Command PDotCommand(double rate) {
    return run(
        () -> {
          if (rate != 0) {
            PDotRate = rate;
            UseButtonState = false;
            PDotPositionCommand = PDotPositionCommand + PDotRate;
            setReference(PDotPositionCommand);
          }
        });
  }

  public Command RateCommand(DoubleSupplier rateSupplier) {
    return run(
        () -> {
          RateCommand = rateSupplier.getAsDouble();
        });
  }

  private void setReference(double position) {
    mReferencePosition = position;
    mDesiredState = new TrapezoidProfile.State(mReferencePosition, 0);
    Logger.recordOutput("arm/Commanded Position", position);
  }

  @Override
  public void periodic() {
    double ffCommand = 0;
    TrapezoidProfile.State state_step = new TrapezoidProfile.State();

    io.updateInputs(inputs);
    Logger.processInputs("arm subsystem", inputs);

    if (simpleControl) {
      ffCommand =
          m_feedforward.calculate(
              Units.degreesToRadians(inputs.armPositionDegrees) - Units.degreesToRadians(90),
              RateCommand);
      io.setVoltage(ffCommand);
    } else {

      if (RobotState.isEnabled() == true) {
        /* Do the command processing */
        state_step = commandProfile.calculate(0.02, mCurrentState, mDesiredState);
        mCurrentState = state_step;
        ffCommand =
            m_feedforward.calculate(
                Units.degreesToRadians(inputs.armPositionDegrees) - Units.degreesToRadians(90),
                Units.degreesToRadians(mCurrentState.velocity));
      } else {
        // do stuff when disabled
        mReferencePosition = 0;
        PDotPositionCommand = mReferencePosition;
        mDesiredState = new TrapezoidProfile.State(mReferencePosition, 0);
        mCurrentState = mDesiredState;
      }
      io.setReference(mCurrentState.position, ffCommand);
    }
    //Logger.recordOutput("arm/speed command", mSpeed);
    Logger.recordOutput("FF", ffCommand);
  }
}
