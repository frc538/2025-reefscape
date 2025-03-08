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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class WristExtenderIOServo implements WristExtenderIO {
  
  public static Servo axonMaxServo; //0 is a placeholder
  private static Servo axonMaxScoringWheel;

  private final DigitalInput SwitchChannel;

  public WristExtenderIOServo(int WristExtenderServoChannel, int ScoringWheelServoChannel, int SwitchDIOChannel) {
    axonMaxServo = new Servo(WristExtenderServoChannel);
    axonMaxScoringWheel = new Servo(ScoringWheelServoChannel);
    SwitchChannel = new DigitalInput(SwitchDIOChannel);

    // if (extendedIsTrue) { //I couldn't get the onTrue function to work
    //   extended = 0; //0 is a placeholder
    //   axonMaxServo.set(extended);
    // } else {
    //   extended = 1; //1 is a placeholder
    //   axonMaxServo.set(extended);
    // }
    // if (controller.back().getAsBoolean() && algaePresent == true) {
    //   //shootAlgae();
    //   algaePresent = false;
    // }
    // if (controller.rightBumper().getAsBoolean() && ) {
    //   //dropCoral();
    //   coralAmount -= 1;
    // }
  }

  @Override
  public void updateInputs(WristExtenderIOInputs inputs) {
    inputs.algaePresent = SwitchChannel.get();
  }
  
  public void goToAngle(double angle) {
    double servoAngle = angle * 0.5 / 90;
    axonMaxServo.setAngle(servoAngle);
  }

  public void intakeAlgaeShootCoral() {
    axonMaxScoringWheel.setSpeed(1.0);
  }
  
  public void intakeCoralShootAlgae() {
    axonMaxScoringWheel.setSpeed(-1.0);
  }
}
