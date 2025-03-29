package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double armPositionDegrees = 0.0;
    public double armVelocityDegreesPerSecond = 0.0;
    public double AppliedOutput = 0.0;
    public double BusVoltage = 0.0;
    public double OutputCurrent = 0.0;
    public double MotorTemperature = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void armSpeedCommand(double speed) {}

  public default void setReference(double position, double vol) {}

  public default void setVoltage(double voltage) {}

  public default void armStop() {}
}
