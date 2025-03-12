package frc.robot.subsystems.limelight;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drive.Drive;

public class limelight extends SubsystemBase {
    public Drive driveFile;
    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(driveFile.kinematics,
            driveFile.rawGyroRotation,
            driveFile.lastModulePositions, new Pose2d());

    public void checkLimelight(String limelightOne) {
        boolean useMegaTag2 = true; // set to false to use MegaTag1
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue(Constants.LimeLightConstants.limelightOneName);
        LimelightHelpers.SetRobotOrientation(Constants.LimeLightConstants.limelightOneName,
                poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0,
                0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLightConstants.limelightOneName);
        if (mt1 != null) {
            Logger.recordOutput("Odometry/MT1/Tag Count", mt1.tagCount);
            Logger.recordOutput("Odometry/MT1/Pose", mt1.pose);
        }
        if (mt2 != null) {
            Logger.recordOutput("Odometry/MT2/Tag Count", mt2.tagCount);
            Logger.recordOutput("Odometry/MT2/Pose", mt2.pose);
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
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                poseEstimator.addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);

            }
        } else if (useMegaTag2 == true) {
            if (Math.abs(driveFile.gyroInputs.yawVelocityRadPerSec) > 720) // if our angular velocity is greater than
                                                                           // 720 degrees per
            // second,
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
            if (!doRejectUpdate) {
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                if (mt2 != null) {
                    poseEstimator.addVisionMeasurement(
                            mt2.pose,
                            mt2.timestampSeconds);
                }
            }
        }

        // Update odometry
        Logger.recordOutput("do reject update", doRejectUpdate);
        double[] sampleTimestamps = driveFile.modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = driveFile.modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - driveFile.lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                driveFile.lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (driveFile.gyroInputs.connected) {
                // Use the real gyro angle
                driveFile.rawGyroRotation = driveFile.gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = driveFile.kinematics.toTwist2d(moduleDeltas);
                driveFile.rawGyroRotation = driveFile.rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], driveFile.rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        driveFile.gyroDisconnectedAlert.set(!driveFile.gyroInputs.connected && Constants.currentMode != Mode.SIM);
    }

    public void simpleUpdateOdometry() {
        var pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimeLightConstants.limelightOneName);
        if (pose == null)
            return;
        poseEstimator.addVisionMeasurement(pose.pose, pose.timestampSeconds);
    }

    @Override
    public void periodic() {
        driveFile.odometryLock.lock(); // Prevents odometry updates while reading data
        driveFile.gyroIO.updateInputs(driveFile.gyroInputs);
        Logger.processInputs("Drive/Gyro", driveFile.gyroInputs);
        for (var module : driveFile.modules) {
            module.periodic();
        }
        driveFile.odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : driveFile.modules) {
                module.stop();
            }
        }

        //checkLimelight(limelightOne);
        // simpleUpdateOdometry();

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }
    }
}
