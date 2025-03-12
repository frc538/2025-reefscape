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
// GNU General Public License for more details.]

// !THIS IS A COPIED FILE OF THE MODULEIO FILE!

package frc.robot.subsystems.WristExtender;

import org.littletonrobotics.junction.AutoLog;

public interface WristExtenderIO {
  @AutoLog
  public static class WristExtenderIOInputs {
    public boolean algaePresent = false;
    public boolean coralPresent = false;
    public double servoPosition = 0.0; // should be same as extended
    public double scoringWheelPosition = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristExtenderIOInputs inputs) {}

  public default void goToPosition(double angle) {}

  public default void intakeAlgaeShootCoral() {}

  public default void intakeCoralShootAlgae() {}

  public default void stopIntake() {}
}
