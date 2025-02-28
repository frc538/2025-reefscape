// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.WristExtender;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.WristExtender.*;
import frc.robot.subsystems.drive.ModuleIO;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Servo;
import org.littletonrobotics.junction.Logger;

/**
 *  Will need imports for Servo and Servo hub, along with anything else that might be important
*/

public class WristExtender extends SubsystemBase {
  private final CommandXboxController controller = new CommandXboxController(0);
  public static Servo axonMaxServo = new Servo(0); //0 is a placeholder
  private static double extended = 1; //1 is just a placeholder
  private boolean extendedIsTrue = controller.start().getAsBoolean();
  public static double algaeAmount = 0;
  public static double coralAmount = 1;
  private final WristExtenderIO io;
  private final WristExtenderIOInputsAutoLogged inputs = new WristExtenderIOInputsAutoLogged();
  private final int index;
  /*axonMaxServo.get(); -gets position
   * axonMaxServo.getAngle(); -gets angle
   * axonMaxServo.set(float); -sets position - 0 is min, 1 is max (Does the same as setAngle, just uses different numbers)
   * axonMaxServo.setAngle(float); -sets angle - 0 is min, 180 is max (Does the same as set, just uses different numbers)
   */
  /** Creates a new ExampleSubsystem. */
  public WristExtender(
      WristExtenderIO io,
      int index) {
    this.io = io;
    this.index = index;
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
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
    /**
     * This is just the base for what the code could look like, just the main functions, nothing else.
     * One question I do have is whether the intake from the elevator will be in this subsystem or the elevator one
     * and how that all will work.
     * From what I've seen there will be two motors, one to extend like an elbow and one like a wrist. 
     * 
     * }
     * public/private void shootAlgae() {
     *  shoot algae? I'm too lazy to think about this rn
     * }
     * public/private void dropCoral() {
     *  another function I am way to lazy to think about smh
     * }
     */
    extendedIsTrue = controller.start().getAsBoolean(); //updating boolean
    //This should mean you have to hold it, not press it once.
    if (extendedIsTrue) { //I couldn't get the onTrue function to work
      extended = 0; //0 is a placeholder
      axonMaxServo.set(extended);
    } else {
      extended = 1; //1 is a placeholder
      axonMaxServo.set(extended);
    }
    if (controller.back().getAsBoolean() && algaeAmount != 0) {
      //shootAlgae();
      algaeAmount -= 1;
    }
    if (controller.rightBumper().getAsBoolean() && coralAmount != 0) {
      //dropCoral();
      coralAmount -= 1;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
