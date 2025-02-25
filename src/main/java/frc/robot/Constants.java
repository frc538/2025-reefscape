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

  public static class ModuleConstants {

    public static int DriveMotorPinionTeeth = 13;
    public static int NeoFreeSpeedRPM = 5676;
    public static double WheelRadiusMeters = Units.inchesToMeters(1.5);
    public static double DriveGearing = 45.0 * 22 / 15 / DriveMotorPinionTeeth;
    public static double TurnPositionConversionFactor = 2 * Math.PI;
    public static double TurnVelocityConversionFactor = 2 * Math.PI / 60;
    public static double RightPositionConversionFactor = 2 * Math.PI;
    public static double RightVelocityConversionFactor = 2 * Math.PI / 60;
    public static double RightMotorFreeSpeedRevsPerSecond = NeoFreeSpeedRPM / 60.0;
    public static double LeftPositionConversionFactor = 2 * Math.PI;
    public static double LeftVelocityConversionFactor = 2 * Math.PI / 60;
    public static double LeftMotorFreeSpeedRevsPerSecond = NeoFreeSpeedRPM / 60.0;
    public static double DrivePositionConversionFactor =
        2 * Math.PI * WheelRadiusMeters / DriveGearing;
    public static double DriveVelocityConversionFactor =
        2 * Math.PI / 60 * WheelRadiusMeters / DriveGearing;
    public static double DriveMotorFreeSpeedRevsPerSecond = NeoFreeSpeedRPM / 60.0;
    public static double DriveWheelFreeSpeedMetersPerSecond =
        DriveMotorFreeSpeedRevsPerSecond * 2.0 * Math.PI * WheelRadiusMeters / DriveGearing;

    public static double DriveP = 0.4;
    public static double DriveI = 0;
    public static double DriveD = 0;
    public static double DriveFF = 1 / DriveWheelFreeSpeedMetersPerSecond;
    public static double DriveMinOutput = -1;
    public static double DriveMaxOutput = 1;

    public static double TurnP = 1;
    public static double TurnI = 0;
    public static double TurnD = 0;
    public static double TurnFF = 0;
    public static double TurnMinOutput = -1;
    public static double TurnMaxOutput = 1;

    public static int DriveCurrentLimit = 50;
    public static int TurnCurrentLimit = 20;

    public static double MaxDriveMetersPerSecond = 4.46;
    public static double MaxTurnRadiansPerSecond = 2 * Math.PI;

    public static boolean DriveEncoderInverted = false;
    public static boolean TurnEncoderInverted = true;

    public static double TurnPIDMinInput = 0;
    public static double TurnPIDMaxInput = 2 * Math.PI;

    public static IdleMode DriveIdle = IdleMode.kBrake;
    public static IdleMode TurnIdle = IdleMode.kBrake;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
