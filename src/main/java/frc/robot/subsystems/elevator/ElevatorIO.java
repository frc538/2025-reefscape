package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        double leftAppliedOutput = 0.0;
        double leftAppliedBusVoltage = 0.0;
        double leftAppliedCurrent = 0.0;
        double rightAppliedOutput = 0.0;
        double rightAppliedBusVoltage = 0.0;
        double rightAppliedCurrent = 0.0;
        double height = 0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {

    }

    public default void setReference(double position) {

    }
}