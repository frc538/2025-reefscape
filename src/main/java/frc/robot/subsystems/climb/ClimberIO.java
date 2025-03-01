package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        boolean ClimberDown = false;
        boolean ClimberUp = false;
    }

    public default void setOutput(double speed){
        
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimberIOInputs inputs) {
    }

}
