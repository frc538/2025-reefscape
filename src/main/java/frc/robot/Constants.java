// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class WristExtenderConstants {
    public static final double coralLowMediumAngle = 45.0;
    public static final double coralHighAngle = 20.0;
    public static final double algaeProcessorAngle = 90.0;
    public static final double bargeAngle = 45.0;
  }
  public static class ClimberConstants {
    public static final int CurrentLimit = 20;
    public static final int ClimberPositionConversionFactor = /*put real value here-->*/ 1;
    public static final int ClimberVelocityConversionFactor = /*put real value here-->*/ 1;
    public static final int ClimberWheelFreeSpeedMetersPerSecond = /*put real value here-->*/ 1;
    public static final int ClimberMotorCANId = 13;
    public static final int IntakeServoDIOID = 2;
  }

  public static class IntakeConstants {
    public static final int InPosition = 819;
    public static final int OutPosition = 1024;
  }
}
