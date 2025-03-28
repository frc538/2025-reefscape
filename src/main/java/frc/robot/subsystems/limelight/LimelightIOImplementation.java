package frc.robot.subsystems.limelight;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;

public class LimelightIOImplementation implements LimelightIO {
    
    private String mLimelightName;

    public LimelightIOImplementation(String LimelightName){
        mLimelightName = LimelightName;
    }
    @Override
    public void updateInputs(LimelightIOInputs inputs, double yawAngleDegrees) {
        inputs.LimelightName = mLimelightName;
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
    }
    
}
