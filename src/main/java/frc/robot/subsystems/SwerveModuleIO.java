package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    
    @AutoLog
    public static class SwerveModuleIOInputs {
        SwerveModuleState state = new SwerveModuleState();
        SwerveModulePosition position = new SwerveModulePosition(); 
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {};

    public default void reset() {};

    public default void setmDesiredState(SwerveModuleState desiredState) {};

}
