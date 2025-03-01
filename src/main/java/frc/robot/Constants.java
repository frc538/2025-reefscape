// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
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

  public static class LimeLightConstants {
    // THESE ARE PRELIMINARY NUMBERS CHANGE LATER
    public static final double limelightX = 2;
    public static final double limelightY = 2;
    public static final double limelightZ = 2;

    public static final double limelightRoll = 0;
    public static final double limelightPitch = -2;
    public static final double limelightYaw = 0;

    public static final String limelightOneName = "limelight-bacon";
    public static final String limelightTwoName = "limelight-eggs";
  }

  public static class ElevatorConstants {

    public static int NeoFreeSpeedRPM = 5676;
    public static double ElevatorPositionConversionFactor = 2 * Math.PI;
    public static double ElevatorVelocityConversionFactor = 2 * Math.PI / 60;
    public static double ElevatorMotorFreeSpeedRevsPerSecond = NeoFreeSpeedRPM / 60.0;

    public static double maxAcceleration = 1;
    public static double maxVelocity = 1;

    public static double allowedClosedLoopError = 0.4;

    public static int ElevatorCurrentLimit = 50;

    public static double arbitraryFeedForward = 1.32;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
