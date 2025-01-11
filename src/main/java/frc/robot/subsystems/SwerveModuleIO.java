package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

  @AutoLog
  public static class SwerveModuleIOInputs {
    SwerveModuleState state = new SwerveModuleState();
    SwerveModulePosition position = new SwerveModulePosition();
  }

  public default void updateInputs(SwerveModuleIOInputs inputs) {}
  ;

  public default void reset() {}
  ;

  public default void setmDesiredState(SwerveModuleState desiredState) {}
  ;
}
