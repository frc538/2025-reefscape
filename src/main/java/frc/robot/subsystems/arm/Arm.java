package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double mSpeed = 0.0;

  public Arm(ArmIO IO) {
    io = IO;
  }

  public Command MoveArm(DoubleSupplier speedSupplier) {
    return run(
        () -> {
          mSpeed = speedSupplier.getAsDouble();
          mSpeed = MathUtil.applyDeadband(mSpeed, 0.1);
          io.armSpeedCommand(mSpeed);
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("arm subsystem", inputs);
    Logger.recordOutput("arm/speed command", mSpeed);
  }
}
