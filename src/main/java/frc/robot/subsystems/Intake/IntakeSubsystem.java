package frc.robot.subsystems.Intake;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final ServoChannel IntakeServo;

  public IntakeSubsystem(ServoHub hub) {
    IntakeServo = hub.getServoChannel(ChannelId.kChannelId1);
    IntakeServo.setPulseWidth(IntakeConstants.InPosition);
    IntakeServo.setPowered(true);
    IntakeServo.setEnabled(true);
  }

  public Command receiveCoral() {
    return runOnce(() -> IntakeServo.setPulseWidth(IntakeConstants.InPosition));
  }

  public Command feedGregory() {
    return runOnce(() -> IntakeServo.setPulseWidth(IntakeConstants.OutPosition));
  }
}
