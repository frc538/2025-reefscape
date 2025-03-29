package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Servo;

public class IntakeIOServo implements IntakeIO{
    private final Servo mServo;

    public IntakeIOServo() {
        mServo.
    }

    public void updateInputs(IntakeIOInputs inputs) {

    }

    public void intakeIn() {
        mServo.setPulseTimeMicroseconds(500);
    }

    public void intakeOut() {
        mServo.setPulseTimeMicroseconds(2500);
    }

    public void intakeStop() {
        mServo.set(0);
    }

    public void intakeHold() {
        mServo.setPulseTimeMicroseconds(1300);
    }
}