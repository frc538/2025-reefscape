// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.servohub.ServoChannel.ChannelId;
import edu.wpi.first.math.util.Units;
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

    /*
     * Rotation conversion: 2*pi*meters(2.074in) m/rotation
     * Gearing: 1/9 output to input
     */

    public static double RotationConversion = 2 * Math.PI * Units.inchesToMeters(2.074);
    public static double ElevatorPositionConversionFactor =
        1.5 / 9.0 * RotationConversion; // m/rotation
    public static double ElevatorVelocityConversionFactor = 1.5 / 9.0 * RotationConversion;
    public static double ElevatorMotorFreeSpeedRevsPerSecond = NeoFreeSpeedRPM / 60.0;

    public static double maxAcceleration = 1;
    public static double maxVelocity = 1;

    public static double allowedClosedLoopError = 0.4;

    public static int ElevatorCurrentLimit = 50;

    public static double arbitraryFeedForward = 0;

    public static int leftCanId = 34;
    public static int rightCanId = 16;

    public static int elevatorUpLimitDIOChannel = 1;
    public static int elevatorDownLimitDIOChannel = 0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ClimberConstants {
    public static final int CurrentLimit = 50;
    public static final double ClimberPositionConversionFactor =
        90.0 / 480.0; // 480 revolutions gives us 90 degrees on the climber.
    public static final int ClimberVelocityConversionFactor = /* put real value here--> */ 1;
    public static final int ClimberWheelFreeSpeedMetersPerSecond = /* put real value here--> */ 1;
    public static final int ClimberMotorCANId = 13;
    public static final int IntakeServoDIOID = 2;
  }

  public static class ArmConstants {
    public static final int ArmCanID = 2;

    public static int NeoFreeSpeedRPM = 5676;
    public static int CurrentLimit = 50;
    public static int gearRatio = 25;
    public static double PositionConversionFactor =
        360.0 / gearRatio; // Arm degrees rotation/motor Revolutions
    public static double VelocityConversionFactor =
        360.0 / gearRatio / 60.0; // Motor RPM to Arm degrees rotation per sec
    public static double MotorFreeSpeedRevsPerSecond = NeoFreeSpeedRPM / 60.0;
  }

  public static class IntakeConstants {
    public static final ChannelId leftChannel = ChannelId.kChannelId0;
    public static final ChannelId rightChannel = ChannelId.kChannelId1;

    public static final int intakeInPulseWidth = 500;
    public static final int intakeOutPulseWidth = 2500;
    public static final int intakeHoldPulseWidth = 1300;
    public static final int initialPulseWidth = 1500;
  }
}
