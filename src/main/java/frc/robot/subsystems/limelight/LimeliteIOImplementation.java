package frc.robot.subsystems.limelight;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;

public class LimeliteIOImplementation implements LimelightIO {
    
    private String mLimelightName;

    public LimeliteIOImplementation(String LimelightName){
        mLimelightName = LimelightName;
    }
    @Override
    public void updateInputs(LimelightIOInputs inputs, double yawAngleDegrees) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue(mLimelightName);
        LimelightHelpers.SetRobotOrientation(mLimelightName,
                yawAngleDegrees, 0,
                0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(mLimelightName);
        if (mt1 != null) {
            inputs.mt1TagCount = mt1.tagCount;
            inputs.mt1Pose = mt1.pose;
            inputs.mt1TimeStamp = mt1.timestampSeconds;
        }
        else{
            inputs.mt1TagCount = 0;
            inputs.mt1Pose = new Pose2d();
            inputs.mt1TimeStamp = 0;
        }
        if (mt2 != null) {
            inputs.mt2TagCount = mt2.tagCount;
            inputs.mt2Pose = mt2.pose;
            inputs.mt2TimeStamp = mt2.timestampSeconds;
        }
        else{
            inputs.mt2TagCount = 0;
            inputs.mt2Pose = mt2.pose;
            inputs.mt2TimeStamp = 0;
        }
        if (useMegaTag2 == false) {

            if (mt1.tagCount == 0) {
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

            Logger.recordOutput("Limelight Pose", mt1.pose);
            if (!doRejectUpdate) {
                driveFile.addVisionMeasurement(
                    mt1.pose, 
                    mt1.timestampSeconds,
                    VecBuilder.fill(.5, .5, 9999999));

            }
        } else if (useMegaTag2 == true) {
            if (Math.abs(driveFile.gyroInputs.yawVelocityRadPerSec) > 720) // if our angular velocity is greater than
                                                                           // 720 degrees per second,
                                                                           // ignore vision updates
            {
                doRejectUpdate = true;
            }
            if (mt2 != null) {
                if (mt2.tagCount == 0) {
                    doRejectUpdate = true;
                }
            }

            Logger.recordOutput("do reject update", doRejectUpdate);
            if (!doRejectUpdate ) {
                if (mt2 != null) {
                    driveFile.addVisionMeasurement(
                        mt2.pose, 
                        mt2.timestampSeconds,
                        VecBuilder.fill(.7, .7, 9999999));
                }
            }
        }

        // Update odometry
        Logger.recordOutput("do reject update", doRejectUpdate);
    }
    
}
