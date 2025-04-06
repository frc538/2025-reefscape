package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double mSpeed = 0.0;
  private boolean algaePresentBoolean = false;

  Constraints profileConstraints;
  TrapezoidProfile commandProfile;
  ArmFeedforward m_feedforward;
  TrapezoidProfile.State mCurrentState;
  TrapezoidProfile.State mDesiredState;

  double maxA = 90;
  double maxV = 45;
  double kS = 0;
  double noAlgaeGain = 0.8; // simple feed forward control
  double withAlgaeGain = 1.9;
  double selectedkG = 0.8;
  double kV = 1;
  double rateGainNoAlgae = 1;
  double rateGainWithAlgae = 2.2;
  double selectedRateGain = 1;

  boolean simpleControl = false; // velocity control loop.
  double RateCommand = 0;

  double PDotPositionCommand = 0;
  double PDotRate = 0;
  boolean UseButtonState = false;
  double mReferencePosition = 0;

  double maxArmRate = 100.00;

  public Arm(ArmIO IO) {
    io = IO;

    profileConstraints = new Constraints(maxV, maxA);
    commandProfile = new TrapezoidProfile(profileConstraints);

    selectedkG = noAlgaeGain; // initialize without algae

    m_feedforward = new ArmFeedforward(kS, selectedkG, kV);
    mCurrentState = new TrapezoidProfile.State();
    mDesiredState = new TrapezoidProfile.State();
  }

  public Command runArmToggle() {
    return runOnce(
        () -> {
          algaePresentBoolean = !algaePresentBoolean;
          if (algaePresentBoolean == true) {
            selectedkG = withAlgaeGain;
            selectedRateGain = rateGainWithAlgae;
          } else {
            selectedkG = noAlgaeGain;
            selectedRateGain = rateGainNoAlgae;
          }
          m_feedforward = new ArmFeedforward(kS, selectedkG, kV);
        });
  }

  public Command MoveArm(DoubleSupplier speedSupplier) {
    return run(
        () -> {
          mSpeed = speedSupplier.getAsDouble();
          mSpeed = MathUtil.applyDeadband(mSpeed, 0.1);
          io.armSpeedCommand(mSpeed);
        });
  }

  /*
   * public Command PDotCommand(double rate) {
   * return run(
   * () -> {
   * if (rate != 0) {
   * PDotRate = rate;
   * UseButtonState = false;
   * PDotPositionCommand = PDotPositionCommand + PDotRate;
   * setReference(PDotPositionCommand);
   * }
   * });
   * }
   */

  public Command RateCommand(DoubleSupplier rateSupplier) {
    return run(
        () -> {
          RateCommand = rateSupplier.getAsDouble();
        });
  }

  /*
   * private void setReference(double position) {
   * mReferencePosition = position;
   * mDesiredState = new TrapezoidProfile.State(mReferencePosition, 0);
   * Logger.recordOutput("arm/Commanded Position", position);
   * }
   */

  @Override
  public void periodic() {
    double ffCommand = 0;

    io.updateInputs(inputs);
    Logger.processInputs("arm subsystem", inputs);

    if (simpleControl) {
      ffCommand =
          m_feedforward.calculate(
              Units.degreesToRadians(inputs.armPositionDegrees) - Units.degreesToRadians(90),
              RateCommand * selectedRateGain);
      io.setVoltage(ffCommand);
    } else {
      /* Do the command processing */
      ffCommand =
          m_feedforward.calculate(
              Units.degreesToRadians(inputs.armPositionDegrees) - Units.degreesToRadians(90),
              RateCommand * selectedRateGain);

      RateCommand = RateCommand * maxArmRate;
      io.setReference(RateCommand, ffCommand);
    }
    // Logger.recordOutput("arm/speed command", mSpeed);
    Logger.recordOutput("arm/ffCommand", ffCommand);
    Logger.recordOutput("arm/RateCommand", RateCommand);
    Logger.recordOutput("arm/selectedkG", selectedkG);
    Logger.recordOutput("arm/Algae Present?", algaePresentBoolean);
  }
}
