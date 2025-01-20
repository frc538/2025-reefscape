// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.SwerveUtils;
import org.littletonrobotics.junction.Logger;

public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveModuleIO mFrontLeftio;
  private final SwerveModuleIO mFrontRightio;
  private final SwerveModuleIO mBackLeftio;
  private final SwerveModuleIO mBackRightio;

  private final SwerveModuleIOInputsAutoLogged mFrontLeftinputs =
      new SwerveModuleIOInputsAutoLogged();
  private final SwerveModuleIOInputsAutoLogged mFrontRightinputs =
      new SwerveModuleIOInputsAutoLogged();
  private final SwerveModuleIOInputsAutoLogged mBackLeftinputs =
      new SwerveModuleIOInputsAutoLogged();
  private final SwerveModuleIOInputsAutoLogged mBackRightinputs =
      new SwerveModuleIOInputsAutoLogged();

  private final Pigeon2 mGyro;
  private final Spark mLights = new Spark(0);
  private boolean mIsFieldRelative = false;

  private double mPreviousTime = WPIUtilJNI.now() * 1e-6;

  private double mCurrentRotation = 0;
  private double mCurrentTranslationDirection = 0;
  private double mCurrentTranslationMagnitude = 0;

  private SlewRateLimiter mMagnitudeLimiter =
      new SlewRateLimiter(Constants.DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter mRotationLimiter =
      new SlewRateLimiter(Constants.DriveConstants.kRotationSlewRate);
  private ChassisSpeeds mSpeedDelivered = new ChassisSpeeds();

  private Pose2d mPose;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveDriveSubsystem(
      SwerveModuleIO FrontLeftDriveio,
      SwerveModuleIO FrontRightDriveio,
      SwerveModuleIO BackLeftDriveio,
      SwerveModuleIO BackRightDriveio,
      int kPigeonID) {

    mFrontLeftio = FrontLeftDriveio;
    mFrontRightio = FrontRightDriveio;
    mBackLeftio = BackLeftDriveio;
    mBackRightio = BackRightDriveio;

    mGyro = new Pigeon2(kPigeonID);
    mGyro.setYaw(0);

    m_odometry =
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(mGyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
              mFrontLeftinputs.position,
              mFrontRightinputs.position,
              mBackLeftinputs.position,
              mBackRightinputs.position
            });
  }

  public void setX() {
    System.out.println("set x");

    mFrontLeftio.setmDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    mFrontRightio.setmDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    mBackLeftio.setmDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    mBackRightio.setmDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    Logger.recordOutput("isSetX true", true);
  }

  public Command setXCommand() {
    return run(() -> setX());
  }

  public Command toggleFieldRelative() {
    return Commands.runOnce(() -> mIsFieldRelative = !mIsFieldRelative, this);
  }

  public Command resetGyroCommand() {
    return Commands.runOnce(() -> mGyro.setYaw(0), this);
  }

  public void drive(double forwardSpeed, double leftSpeed, double rotation, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    Logger.recordOutput("Drive/Forward speed", forwardSpeed);
    Logger.recordOutput("Drive/Left speed", leftSpeed);
    Logger.recordOutput("Drive/Rotation speed", rotation);

    if (rateLimit) {

      double inputTranslationDirection = Math.atan2(leftSpeed, forwardSpeed);
      double inputTranslationMagnitude =
          Math.sqrt(forwardSpeed * forwardSpeed + leftSpeed * leftSpeed);

      double directionSlewRate;
      if (mCurrentTranslationMagnitude != 0) {
        directionSlewRate =
            Math.abs(Constants.DriveConstants.kDirectionSlewRate / mCurrentTranslationMagnitude);
      } else {
        directionSlewRate = 500.0;
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - mPreviousTime;
      double angleDifference =
          SwerveUtils.angleDifference(inputTranslationDirection, mCurrentTranslationDirection);

      if (angleDifference < 0.45 * Math.PI) {
        mCurrentTranslationDirection =
            SwerveUtils.stepTowardsCircular(
                mCurrentTranslationDirection,
                inputTranslationDirection,
                directionSlewRate * elapsedTime);
        mCurrentTranslationMagnitude = mMagnitudeLimiter.calculate(inputTranslationMagnitude);
      } else if (angleDifference > 0.85 * Math.PI) {
        if (mCurrentTranslationMagnitude > 1e-4) {
          mCurrentTranslationMagnitude = mMagnitudeLimiter.calculate(0);
        } else {
          mCurrentTranslationDirection =
              SwerveUtils.wrapAngle(mCurrentTranslationDirection + Math.PI);
          mCurrentTranslationMagnitude = mMagnitudeLimiter.calculate(inputTranslationMagnitude);
        }
      } else {
        mCurrentTranslationDirection =
            SwerveUtils.stepTowardsCircular(
                mCurrentTranslationDirection,
                inputTranslationDirection,
                directionSlewRate * elapsedTime);
        mCurrentTranslationMagnitude = mMagnitudeLimiter.calculate(0);
      }

      mPreviousTime = currentTime;

      xSpeedCommanded = mCurrentTranslationMagnitude * Math.cos(mCurrentTranslationDirection);
      ySpeedCommanded = mCurrentTranslationMagnitude * Math.sin(mCurrentTranslationDirection);
      mCurrentRotation = mRotationLimiter.calculate(rotation);

    } else {
      xSpeedCommanded = forwardSpeed;
      ySpeedCommanded = leftSpeed;
      mCurrentRotation = rotation;
    }

    double xSpeedDelivered = xSpeedCommanded * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double rotationDelivered = mCurrentRotation * Constants.DriveConstants.kMaxAngularSpeed;

    mSpeedDelivered = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered);

    var swerveModuleStates =
        Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
            mIsFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotationDelivered,
                    Rotation2d.fromDegrees(mGyro.getYaw().getValueAsDouble()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    mFrontLeftio.setmDesiredState(swerveModuleStates[0]);
    mFrontRightio.setmDesiredState(swerveModuleStates[1]);
    mBackLeftio.setmDesiredState(swerveModuleStates[2]);
    mBackRightio.setmDesiredState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mFrontLeftio.updateInputs(mFrontLeftinputs);
    mFrontRightio.updateInputs(mFrontRightinputs);
    mBackLeftio.updateInputs(mBackLeftinputs);
    mBackRightio.updateInputs(mBackRightinputs);

    Logger.processInputs("Drive/Front Left", mFrontLeftinputs);
    Logger.processInputs("Drive/Front Right", mFrontRightinputs);
    Logger.processInputs("Drive/Back Left", mBackLeftinputs);
    Logger.processInputs("Drive/Back Right", mBackRightinputs);

    SwerveModuleState[] states =
        new SwerveModuleState[] {
          mFrontLeftinputs.state,
          mFrontRightinputs.state,
          mBackLeftinputs.state,
          mBackRightinputs.state
        };

    Logger.recordOutput("MyStates", states);

    Logger.recordOutput("Is Field Relative?", mIsFieldRelative);
    Logger.recordOutput("Gyro Angle", mGyro.getYaw().getValueAsDouble());
    mLights.set(mIsFieldRelative ? 0.77 : 0.61);

    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(mGyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
          mFrontLeftinputs.position,
          mFrontRightinputs.position,
          mBackLeftinputs.position,
          mBackRightinputs.position
        });
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(mGyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
          mFrontLeftio.getPosition(),
          mFrontRightio.getPosition(),
          mBackLeftio.getPosition(),
          mBackRightio.getPosition()
        },
        pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return mSpeedDelivered;
  }
}
