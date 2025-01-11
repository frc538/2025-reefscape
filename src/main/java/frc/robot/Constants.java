// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;
  public static final Mode currentMode = Mode.REAL;

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

  public static class DriveConstants {
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // Mesured in percent per second (1 = 100%)
    public static final double kRotationSlewRate = 2.0; // Mesured in percent per second (1 = 100%)
  }

  public static class SparkMaxCANID {
    public static final int kFrontRightDrive = 23;
    public static final int kFrontRightTurn = 15;
    public static final int kFrontLeftDrive = 12;
    public static final int kFrontLeftTurn = 9;
    public static final int kRearRightDrive = 22;
    public static final int kRearRightTurn = 1;
    public static final int kRearLeftDrive = 11;
    public static final int kRearLefttTurn = 2;
  }

  public static class ModuleConstants {
    public static int DriveMotorPinionTeeth = 13;
    public static int NeoFreeSpeedRPM = 5676;
    public static double WheelRadiusMeters = Units.inchesToMeters(1.5);
    public static double DriveGearing = 45.0 * 22 / 15 / DriveMotorPinionTeeth;
    public static double TurnPositionConversionFactor = 2 * Math.PI;
    public static double TurnVelocityConversionFactor = 2 * Math.PI / 60;
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

    public static double TrackWidth = Units.inchesToMeters(25.5);
    public static double WheelBase = Units.inchesToMeters(25.5);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(WheelBase / 2, TrackWidth / 2),
            new Translation2d(WheelBase / 2, -TrackWidth / 2),
            new Translation2d(-WheelBase / 2, TrackWidth / 2),
            new Translation2d(-WheelBase / 2, -TrackWidth / 2));

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
}
