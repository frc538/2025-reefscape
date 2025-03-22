package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {

    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {
    }
}
