package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimberIO;

public class Intake extends SubsystemBase {
    IntakeIO io;

    public Intake(IntakeIO IO) {
        io = IO;
    }

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    Servo servoLeft = new Servo(1);
    Servo servoRight = new Servo(0);

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("intake subsystem", inputs);
    }
}
