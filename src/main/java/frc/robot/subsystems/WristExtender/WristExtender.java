// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WristExtender;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.WristExtender.*;
import frc.robot.subsystems.drive.ModuleIO;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Servo;
import org.littletonrobotics.junction.Logger;

/**
 *  Will need imports for Servo and Servo hub, along with anything else that might be important
 *  None of this is imported into Robot/robotcontainer yet.
*/

public class WristExtender extends SubsystemBase {
  private final WristExtenderIO io;
  private final WristExtenderIOInputsAutoLogged inputs = new WristExtenderIOInputsAutoLogged();
  /*axonMaxServo.get(); -gets position
   * axonMaxServo.getAngle(); -gets angle
   * axonMaxServo.set(float); -sets position - 0 is min, 1 is max (Does the same as setAngle, just uses different numbers)
   * axonMaxServo.setAngle(float); -sets angle - 0 is min, 180 is max (Does the same as set, just uses different numbers)
   */
  /** Creates a new ExampleSubsystem. */
  public WristExtender(
      WristExtenderIO io) {
    this.io = io;
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */

  public Command intakeAlgaeShootCoral() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          io.intakeAlgaeShootCoral();
        });
  }

  public Command intakeCoralShootAlgae() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          io.intakeCoralShootAlgae();
        });
  }

  public Command goToCoralReefLowMedium() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          io.goToAngle(Constants.WristExtenderConstants.coralLowMediumAngle);
        });
  }

  
  public Command goToCoralReefHigh() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          io.goToAngle(Constants.WristExtenderConstants.coralHighAngle);
        });
  }
  
  public Command goToAlgaeProcessor() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          io.goToAngle(Constants.WristExtenderConstants.algaeProcessorAngle);
        });
  }

  
  public Command goToBarge() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          io.goToAngle(Constants.WristExtenderConstants.bargeAngle);
        });
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    /*
     * From what I've seen there will be two motors, one to extend like an elbow and one like a wrist. 
    */

    io.updateInputs(inputs);
    Logger.processInputs("Wrist Extender", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
