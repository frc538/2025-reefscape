package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double upSpeed = 0.1;
  private double downSpeed = -0.1;

  public Arm(ArmIO IO) {
    io = IO;
  }

  public Command ArmUp() {
    return run(() -> io.armSpeedCommand(upSpeed));
  }

  public Command ArmDown() {
    return run(() -> io.armSpeedCommand(downSpeed));
  }

  public Command ArmStop() {
    return run(() -> io.armStop());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("arm subsystem", inputs);
  }
}
