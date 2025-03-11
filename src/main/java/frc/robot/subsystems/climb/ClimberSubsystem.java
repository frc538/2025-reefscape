package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem(ClimberIO IO) {
    io = IO;
  }

  public Command ClimberUp() {
    return runEnd(() -> io.setOutput(-0.5), () -> io.setOutput(0.0));
  }

  public Command ClimberDown() {
    return runEnd(() -> io.setOutput(0.5), () -> io.setOutput(0.0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("climber subsystem", inputs);
  }
}
