package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    SwerveModuleState state = new SwerveModuleState();
    SwerveModulePosition position = new SwerveModulePosition();
    double driveAppliedOutput = 0.0;
    double driveAppliedBusVoltage = 0.0;
    double driveAppliedCurrent = 0.0;
    double turnAppliedOutput = 0.0;
    double turnAppliedBusVoltage = 0.0;
    double turnAppliedCurrent = 0.0;
  }

  public default void updateInputs(SwerveModuleIOInputs inputs) {}
  ;

  public default void reset() {}
  ;

  public default void setmDesiredState(SwerveModuleState desiredState) {}
  ;

  public default SwerveModulePosition getPosition() {
    return null;
  }
}
