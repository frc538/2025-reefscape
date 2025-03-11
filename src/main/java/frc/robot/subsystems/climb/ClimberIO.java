package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean climberDown = false;
    public boolean climberUp = false;
    public double motorPosition = 0.0;
    public double motorVoltage = 0;
    public double motorCurrent = 0;
  }

  public default void setOutput(double speed) {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}
}
