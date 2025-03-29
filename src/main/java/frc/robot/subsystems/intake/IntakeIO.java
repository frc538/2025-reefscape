package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {
    }
}
