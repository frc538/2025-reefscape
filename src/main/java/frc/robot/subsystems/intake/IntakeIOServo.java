package frc.robot.subsystems.intake;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;

import frc.robot.Constants;

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
    servoLeft.setPulseWidth(Constants.IntakeConstants.intakeInPulseWidth);
    servoRight.setPulseWidth(Constants.IntakeConstants.intakeInPulseWidth);
  }

  public void intakeOut() {
    servoLeft.setPulseWidth(Constants.IntakeConstants.intakeOutPulseWidth);
    servoRight.setPulseWidth(Constants.IntakeConstants.intakeOutPulseWidth);
  }

  public void intakeStop() {
    servoLeft.setPulseWidth(Constants.IntakeConstants.intakeHoldPulseWidth);
    servoRight.setPulseWidth(Constants.IntakeConstants.intakeHoldPulseWidth);
  }

  public void intakeResetOff() {
    servoLeft.setPowered(false);
    servoRight.setPowered(false);
  }

  public void intakeResetOn() {
    servoLeft.setPowered(true);
    servoRight.setPowered(true);
  }
}
