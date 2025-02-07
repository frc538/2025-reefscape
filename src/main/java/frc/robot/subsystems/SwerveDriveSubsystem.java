// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.SwerveUtils;
import frc.robot.subsystems.NavgationIO.GyroIOInputs;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveModuleIO mFrontLeftio;
  private final SwerveModuleIO mFrontRightio;
  private final SwerveModuleIO mBackLeftio;
  private final SwerveModuleIO mBackRightio;

  private final NavgationIO mGyroIO;

  private final SwerveModuleIOInputsAutoLogged mFrontLeftinputs = new SwerveModuleIOInputsAutoLogged();
  private final SwerveModuleIOInputsAutoLogged mFrontRightinputs = new SwerveModuleIOInputsAutoLogged();
  private final SwerveModuleIOInputsAutoLogged mBackLeftinputs = new SwerveModuleIOInputsAutoLogged();
  private final SwerveModuleIOInputsAutoLogged mBackRightinputs = new SwerveModuleIOInputsAutoLogged();

  private final GyroIOInputs mGyroIOInputs = new GyroIOInputs();

  private final Spark mLights = new Spark(0);
  private boolean mIsFieldRelative = false;

  private Pose2d botPoseLimelight = LimelightHelpers
      .getBotPose2d_wpiBlue(Constants.LimeLightConstants.limelightOneName);

  private double mPreviousTime = WPIUtilJNI.now() * 1e-6;

  private double mCurrentRotation = 0;
  private double mCurrentTranslationDirection = 0;
  private double mCurrentTranslationMagnitude = 0;

  private SlewRateLimiter mMagnitudeLimiter = new SlewRateLimiter(Constants.DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter mRotationLimiter = new SlewRateLimiter(Constants.DriveConstants.kRotationSlewRate);
  private ChassisSpeeds mSpeedDelivered = new ChassisSpeeds();

  private Pose2d mPose;

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry;

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveDriveSubsystem(
      SwerveModuleIO FrontLeftDriveio,
      SwerveModuleIO FrontRightDriveio,
      SwerveModuleIO BackLeftDriveio,
      SwerveModuleIO BackRightDriveio,
      NavgationIO gyroIO) {

    mFrontLeftio = FrontLeftDriveio;
    mFrontRightio = FrontRightDriveio;
    mBackLeftio = BackLeftDriveio;
    mBackRightio = BackRightDriveio;

    mGyroIO = gyroIO;

    m_odometry = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(mGyroIOInputs.yaw),
        new SwerveModulePosition[] {
            mFrontLeftinputs.position,
            mFrontRightinputs.position,
            mBackLeftinputs.position,
            mBackRightinputs.position
        }, LimelightHelpers.getBotPose2d_wpiBlue(Constants.LimeLightConstants.limelightOneName));
  }

  // m_odometry.addVisionMeasurement(botPoseLimelight,
  // edu.wpi.first.wpilibj.Timer.getFPGATimestamp());

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

  private void toggleFieldRelative() {
    System.out.println("toggle field relative.");
    mIsFieldRelative = !mIsFieldRelative;
  }

  public Command toggleFieldRelativeCommand() {
    return runOnce(this::toggleFieldRelative).ignoringDisable(true);
  }

  public Command resetGyroCommand() {
    return Commands.runOnce(() -> mGyroIO.setYaw(0), this);
  }

  public void drive(double forwardSpeed, double leftSpeed, double rotation, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    Logger.recordOutput("Drive/Forward speed", forwardSpeed);
    Logger.recordOutput("Drive/Left speed", leftSpeed);
    Logger.recordOutput("Drive/Rotation speed", rotation);

    if (rateLimit) {

      double inputTranslationDirection = Math.atan2(leftSpeed, forwardSpeed);
      double inputTranslationMagnitude = Math.sqrt(forwardSpeed * forwardSpeed + leftSpeed * leftSpeed);

      double directionSlewRate;
      if (mCurrentTranslationMagnitude != 0) {
        directionSlewRate = Math.abs(Constants.DriveConstants.kDirectionSlewRate / mCurrentTranslationMagnitude);
      } else {
        directionSlewRate = 500.0;
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - mPreviousTime;
      double angleDifference = SwerveUtils.angleDifference(inputTranslationDirection, mCurrentTranslationDirection);

      if (angleDifference < 0.45 * Math.PI) {
        mCurrentTranslationDirection = SwerveUtils.stepTowardsCircular(
            mCurrentTranslationDirection,
            inputTranslationDirection,
            directionSlewRate * elapsedTime);
        mCurrentTranslationMagnitude = mMagnitudeLimiter.calculate(inputTranslationMagnitude);
      } else if (angleDifference > 0.85 * Math.PI) {
        if (mCurrentTranslationMagnitude > 1e-4) {
          mCurrentTranslationMagnitude = mMagnitudeLimiter.calculate(0);
        } else {
          mCurrentTranslationDirection = SwerveUtils.wrapAngle(mCurrentTranslationDirection + Math.PI);
          mCurrentTranslationMagnitude = mMagnitudeLimiter.calculate(inputTranslationMagnitude);
        }
      } else {
        mCurrentTranslationDirection = SwerveUtils.stepTowardsCircular(
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

    var swerveModuleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
        mIsFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotationDelivered,
                Rotation2d.fromDegrees(mGyroIOInputs.yaw))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    mFrontLeftio.setmDesiredState(swerveModuleStates[0]);
    mFrontRightio.setmDesiredState(swerveModuleStates[1]);
    mBackLeftio.setmDesiredState(swerveModuleStates[2]);
    mBackRightio.setmDesiredState(swerveModuleStates[3]);

    Logger.recordOutput("Drive/Commanded Swerve State Arrays", swerveModuleStates);
  }

  public void UpdateOdometry() {
    boolean useMegaTag2 = true; // set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if (useMegaTag2 == false) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if (mt1.rawFiducials[0].ambiguity > .7) {
          doRejectUpdate = true;
        }
        if (mt1.rawFiducials[0].distToCamera > 3) {
          doRejectUpdate = true;
        }
      }
      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {
        m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        m_odometry.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    } else if (useMegaTag2 == true) {
      LimelightHelpers.SetRobotOrientation("limelight", m_odometry.getEstimatedPosition().getRotation().getDegrees(), 0,
          0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
          .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLightConstants.limelightOneName);
      if (Math.abs(mGyroIOInputs.yawRate) > 720) // if our angular velocity is greater than 720 degrees per second,
                                                 // ignore vision updates
      {
        doRejectUpdate = true;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        m_odometry.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
    m_odometry.update(
        Rotation2d.fromDegrees(mGyroIOInputs.yaw),
        new SwerveModulePosition[] {
            mFrontLeftinputs.position,
            mFrontRightinputs.position,
            mBackLeftinputs.position,
            mBackRightinputs.position
        });
  }

  @Override
  public void periodic() {
    mGyroIO.updateInputs(mGyroIOInputs);
    // This method will be called once per scheduler run
    mFrontLeftio.updateInputs(mFrontLeftinputs);
    mFrontRightio.updateInputs(mFrontRightinputs);
    mBackLeftio.updateInputs(mBackLeftinputs);
    mBackRightio.updateInputs(mBackRightinputs);

    botPoseLimelight = LimelightHelpers.getBotPose2d_wpiBlue(Constants.LimeLightConstants.limelightOneName);

    Logger.processInputs("Drive/Front Left", mFrontLeftinputs);
    Logger.processInputs("Drive/Front Right", mFrontRightinputs);
    Logger.processInputs("Drive/Back Left", mBackLeftinputs);
    Logger.processInputs("Drive/Back Right", mBackRightinputs);

    Logger.recordOutput("LimeLight Pose", botPoseLimelight);

    SwerveModuleState[] states = new SwerveModuleState[] {
        mFrontLeftinputs.state,
        mFrontRightinputs.state,
        mBackLeftinputs.state,
        mBackRightinputs.state
    };

    Logger.recordOutput("Drive/moduleStates", states);

    Logger.recordOutput("Is Field Relative?", mIsFieldRelative);
    Logger.recordOutput("Gyro Angle", mGyroIOInputs.yaw);

    var pose = m_odometry.getEstimatedPosition();

    Logger.recordOutput("Odometry X", pose == null ? 0 : pose.getX());
    Logger.recordOutput("Odometry Y", pose == null ? 0 : pose.getY());
    Logger.recordOutput("Drive/OdometryPose", pose);

    mLights.set(mIsFieldRelative ? 0.77 : 0.61);
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(mGyroIOInputs.yaw),
        new SwerveModulePosition[] {
            mFrontLeftinputs.position,
            mFrontRightinputs.position,
            mBackLeftinputs.position,
            mBackRightinputs.position
        },
        pose);

    UpdateOdometry();
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return mSpeedDelivered;
  }
}
