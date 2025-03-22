package frc.robot.subsystems.limelight;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drive.Drive;

public class Limelight extends SubsystemBase {
    public Drive driveFile;

    public Limelight(Drive driveFile) {  //I don't know if this was needed or not
        this.driveFile = driveFile;
        ;
    }

    public void checkLimelight(String limelightOne) {
        boolean useMegaTag2 = true; // set to false to use MegaTag1
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue(limelightOne);
        LimelightHelpers.SetRobotOrientation(limelightOne,
                driveFile.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0,
                0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(limelightOne);
        if (mt1 != null) {
            Logger.recordOutput(String.format("Odometry/%s-MT1/Tag Count",limelightOne), mt1.tagCount);
            Logger.recordOutput(String.format("Odometry/%s-MT1/Pose", limelightOne), mt1.pose);
        }
        if (mt2 != null) {
            Logger.recordOutput(String.format("Odometry/%s-MT2/Tag Count", limelightOne), mt2.tagCount);
            Logger.recordOutput(String.format("Odometry/%s-MT2/Pose", limelightOne), mt2.pose);
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

    @Override
    public void periodic() {
        checkLimelight(Constants.LimeLightConstants.limelightOneName);
        checkLimelight(Constants.LimeLightConstants.limelightTwoName);
    }
}