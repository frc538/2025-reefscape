package frc.robot.subsystems.intake;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;

public class IntakeIOServo implements IntakeIO {
  private final ServoChannel servoLeft; // 0 is a placeholder
  private final ServoChannel servoRight;

  public IntakeIOServo(ServoHub hub, ChannelId servoLeftChannelId, ChannelId servoRightChannelId) {
    servoLeft = hub.getServoChannel(servoLeftChannelId);
    servoRight = hub.getServoChannel(servoRightChannelId);

    servoRight.setPowered(true);
    servoRight.setPulseWidth(1500);
    servoRight.setEnabled(false);

    servoLeft.setPowered(true);
    servoLeft.setPulseWidth(500);
    servoLeft.setEnabled(false);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.servoLeftPulseWidth = servoLeft.getPulseWidth();
    inputs.servoRightPulseWidth = servoRight.getPulseWidth();
  }

  @Override
  public void intakeIn() {
    servoLeft.setPulseWidth(500);
    servoRight.setPulseWidth(500);
  }

  public void intakeOut() {
    servoLeft.setPulseWidth(2500);
    servoRight.setPulseWidth(2500);
  }

  public void intakeStop() {
    servoLeft.setPulseWidth(0);
    servoRight.setPulseWidth(0);
  }

  public void intakeHold() {
    servoLeft.setPulseWidth(1300);
    servoRight.setPulseWidth(1300);
  }
}
