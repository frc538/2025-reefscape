// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// !THIS IS A COPIED FILE OF THE MODULEIOTALONFX FILE!

package frc.robot.subsystems.WristExtender;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class WristExtenderIOServo implements WristExtenderIO {

  private final ServoChannel axonMaxServo; // 0 is a placeholder
  private final ServoChannel axonMaxScoringWheel;
  private final DigitalInput SwitchChannelAlgae; // 0 is a placeholder
  private final DigitalInput SwitchChannelCoral;

  public WristExtenderIOServo(
      ServoHub hub,
      ChannelId WristExtenderServoChannel,
      ChannelId ScoringWheelServoChannel,
      int SwitchDIOChannelAlgae,
      int SwitchDIOChannelCoral) {
    axonMaxServo = hub.getServoChannel(WristExtenderServoChannel);
    axonMaxScoringWheel = hub.getServoChannel(ScoringWheelServoChannel);

    axonMaxScoringWheel.setPowered(true);
    axonMaxScoringWheel.setPulseWidth(1500);
    axonMaxScoringWheel.setEnabled(false);

    axonMaxServo.setPowered(true);
    axonMaxServo.setPulseWidth(1500);
    axonMaxServo.setEnabled(false);

    SwitchChannelAlgae = new DigitalInput(SwitchDIOChannelAlgae);
    SwitchChannelCoral = new DigitalInput(SwitchDIOChannelCoral);
  }

  @Override
  public void updateInputs(WristExtenderIOInputs inputs) {
    inputs.algaePresent = SwitchChannelAlgae.get();
    inputs.coralPresent = SwitchChannelCoral.get();
    inputs.wheelPulseWidth = axonMaxScoringWheel.getPulseWidth();
    inputs.servoPulseWidth = axonMaxServo.getPulseWidth();
  }

  @Override
  public void goToPosition(int pulseWidth) {
    Logger.recordOutput("Gregory/goToPosition", pulseWidth);
    axonMaxServo.setPulseWidth(pulseWidth);
    axonMaxServo.setEnabled(true);
  }

  public void intakeAlgaeShootCoral() {
    axonMaxScoringWheel.setPulseWidth(2500);
    axonMaxScoringWheel.setEnabled(true);
  }

  public void intakeCoralShootAlgae() {
    axonMaxScoringWheel.setPulseWidth(500);
    axonMaxScoringWheel.setEnabled(true);
  }

  public void stopIntake() {
    axonMaxScoringWheel.setEnabled(false);
  }
}
