package frc.robot.subsystems.Intake;

import java.io.InputStream;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.climb.ClimberIO.ClimberIOInputs;

public class IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public double ServoPos = 0.0;
    }
}
