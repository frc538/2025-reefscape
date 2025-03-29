package frc.robot.subsystems.limelight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.limelight.LimelightIO.LimelightIOInputs;

import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {
  public Drive driveFile;
  private LimelightIO[] mLimelightIOs;
  private LimelightIOInputsAutoLogged[] mLimelightIOInputs = new LimelightIOInputsAutoLogged[2];

  public Limelight(
      Drive driveFile, LimelightIO[] Limelights) { // I don't know if this was needed or not
    this.driveFile = driveFile;
    mLimelightIOs = Limelights;
    mLimelightIOInputs[0] = new LimelightIOInputsAutoLogged();
    mLimelightIOInputs[1] = new LimelightIOInputsAutoLogged();
  }

  public void checkLimelight(int LimelightIndex) {
    boolean useMegaTag2 = true; // set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if (useMegaTag2 == false) {

      if (mLimelightIOInputs[LimelightIndex].mt1TagCount == 0) {
        doRejectUpdate = true;
      }
      /*
       * if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
       * if (mt1.rawFiducials[0].ambiguity > .7) {
       * doRejectUpdate = true;
       * }
       * if (mt1.rawFiducials[0].distToCamera > 3) {
       * doRejectUpdate = true;
       * }
       * }
       */

      Logger.recordOutput("Limelight Pose", mLimelightIOInputs[LimelightIndex].mt1Pose);
      if (!doRejectUpdate) {
        driveFile.addVisionMeasurement(
            mLimelightIOInputs[LimelightIndex].mt1Pose,
            mLimelightIOInputs[LimelightIndex].mt1TimeStamp,
            VecBuilder.fill(.5, .5, 9999999));
      }
    } else if (useMegaTag2 == true) {
      if (Math.abs(driveFile.gyroInputs.yawVelocityRadPerSec)
          > 720) // if our angular velocity is greater than
      // 720 degrees per second,
      // ignore vision updates
      {
        doRejectUpdate = true;
      }
      if (mLimelightIOInputs[LimelightIndex] != null) {
        if (mLimelightIOInputs[LimelightIndex].mt2TagCount == 0) {
          doRejectUpdate = true;
        }
      }

      Logger.recordOutput("do reject update", doRejectUpdate);
      if (!doRejectUpdate) {
        if (mLimelightIOInputs[LimelightIndex] != null) {
          driveFile.addVisionMeasurement(
              mLimelightIOInputs[LimelightIndex].mt2Pose,
              mLimelightIOInputs[LimelightIndex].mt2TimeStamp,
              VecBuilder.fill(.7, .7, 9999999));
        }
      }
    }

    // Update odometry
    Logger.recordOutput("do reject update", doRejectUpdate);
  }

  @Override
  public void periodic() {

    mLimelightIOs[0].updateInputs(mLimelightIOInputs[0], driveFile.getRotation().getDegrees());
    mLimelightIOs[1].updateInputs(mLimelightIOInputs[1], driveFile.getRotation().getDegrees());
    Logger.processInputs(mLimelightIOInputs[0].LimelightName, mLimelightIOInputs[0]);
    Logger.processInputs(mLimelightIOInputs[1].LimelightName, mLimelightIOInputs[1]);

    checkLimelight(0);
    checkLimelight(1);
  }
}
