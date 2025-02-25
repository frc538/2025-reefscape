package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {

    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {
        
    }
}