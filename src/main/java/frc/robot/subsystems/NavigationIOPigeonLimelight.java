package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;

public class NavigationIOPigeonLimelight implements NavgationIO {

  private final Pigeon2 mGyro;

  public NavigationIOPigeonLimelight(int kPigeonID) {
    mGyro = new Pigeon2(kPigeonID);
    mGyro.setYaw(0);
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.yaw = mGyro.getYaw().getValueAsDouble();
  }

  public void setYaw(double yaw) {
    mGyro.setYaw(yaw);
  }

  public void limelightPose(Pose2d limelightPose2d) {
    
  }
}
