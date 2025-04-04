package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  double mSpeed = 0;

  public ClimberSubsystem(ClimberIO IO) {
    io = IO;
  }

  public Command ClimberUp() {
    return runEnd(
        () -> {
          mSpeed = -1;
          io.setOutput(mSpeed);
        },
        () -> {
          io.setOutput(0.0);
          mSpeed = 0;
        });
  }

  public Command ClimberSpeed(DoubleSupplier speedSupplier) {
    return run(
        () -> {
          mSpeed = speedSupplier.getAsDouble();
          io.setOutput(mSpeed);
        });
  }

  public Command ClimberDown() {
    return runEnd(
        () -> {
          mSpeed = 1;
          io.setOutput(mSpeed);
        },
        () -> {
          io.setOutput(0.0);
          mSpeed = 0;
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("climber subsystem", inputs);
    Logger.recordOutput("climber subsystem/mSpeed", mSpeed);
  }
}
