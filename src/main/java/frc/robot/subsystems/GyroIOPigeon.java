package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon implements GyroIO {

  private final Pigeon2 mGyro;

  public GyroIOPigeon(int kPigeonID) {
    mGyro = new Pigeon2(kPigeonID);
    mGyro.setYaw(0);
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.yaw = mGyro.getYaw().getValueAsDouble();
  }

  public void setYaw(double yaw) {
    mGyro.setYaw(yaw);
  }
}
