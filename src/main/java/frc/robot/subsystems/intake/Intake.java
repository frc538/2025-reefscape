package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    Servo servoLeft = new Servo(1);
    Servo servoRight = new Servo(0);

    @Override
    public void periodic() {

    }
}
