// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

/**
 *  Will need imports for Servo and Servo hub, along with anything else that might be important
*/

public class WristExtender extends SubsystemBase {
  private Servo axonMaxServo;
  /*axonMaxServo.get(); -gets position
   * axonMaxServo.getAngle(); -gets angle
   * axonMaxServo.set(float); -sets position
   * axonMaxServo.setAngle(float); -sets angle
   */
  /** Creates a new ExampleSubsystem. */
  public WristExtender() {}
  //private static final double extended = 0.5;
  //Will find a way to get whether we have coral/algae in our system or not and how many.
  //get boolean algaeAmount
  //get boolean coralAmount
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
     * If [button] is true/pressed (not held) { - We can condense these commands into an automated sequence if needed
     *  while [button1] is pressed and extended != 1, extended += 0.5, else return nothing
     *  while [button2] is pressed and extended != 0, extended -= 0.5, else return nothing - these can be changed to if trigger/stick is 
     * negative/positive, add or subtract or return nothing from extend. I do think a button would fit better, though.
     * It's pretty choppy, though, so lowering the increments might be good.
     *  axonMaxServo.set(extend);
     *  if [button3] is pressed, shootAlgae() and algaeAmount = false;
     *  if [button4] is pressed, dropCoral() and coralAmount = false;
     * }
     */
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
