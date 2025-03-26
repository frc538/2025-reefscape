package frc.robot.subsystems.limelight;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface LimelightIO {
    
        @AutoLog
    public static class LimelightIOInputs{

        public int mt1TagCount = 0;
        public Pose2d mt1Pose = new Pose2d();
        public double mt1TimeStamp = 0;
        
        public int mt2TagCount = 0;
        public Pose2d mt2Pose = new Pose2d();
        public double mt2TimeStamp = 0;
    }
    public default void updateInputs(LimelightIOInputs inputs, double yawAngleDegrees) {}

}
