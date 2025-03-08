package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        double leftOutput = 0.0;
        double leftBusVoltage = 0.0;
        double leftCurrent = 0.0;
        double leftEncoderValue = 0.0;
        double rightEncoderValue = 0.0;
        double rightOutput = 0.0;
        double rightBusVoltage = 0.0;
        double rightCurrent = 0.0;
        double height = 0;

        boolean atBottom = false;
        boolean atTop = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {

    }

    public default void resetEncoders() {
        
    }

    public default void setReference(double position) {

    }

    public default void setArbFF(double arbFF) {}

    public default void commandMotor() {}
}