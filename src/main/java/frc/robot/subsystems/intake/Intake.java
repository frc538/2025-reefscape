package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  IntakeIO io;

  public Intake(IntakeIO IO) {
    io = IO;
  }

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Command intakeIn() {
    return run(
        () -> {
          io.intakeIn();
        });
  }

  public Command intakeOut() {
    return run(
        () -> {
          io.intakeOut();
        });
  }

  public Command intakeStop() {
    return run(
        () -> {
          io.intakeStop();
        });
  }

  public Command intakeReset() {
    return runEnd(() -> {
        io.intakeResetOn();
    }, () -> {
        io.intakeResetOff();
    });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("intake subsystem", inputs);
  }
}
