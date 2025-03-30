package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public int servoLeftPulseWidth = 0;
    public int servoRightPulseWidth = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void intakeIn() {}

  public default void intakeOut() {}

  public default void intakeStop() {}

  public default void intakeResetOff() {}

  public default void intakeResetOn() {}
}
