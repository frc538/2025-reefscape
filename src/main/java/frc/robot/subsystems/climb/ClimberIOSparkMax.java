package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ClimberIOSparkMax implements ClimberIO {
  private final SparkMax motor;
  private final SparkRelativeEncoder motorRelativeEncoder;
  private final DigitalInput LimitSwitchBottom;
  private final DigitalInput LimitSwitchTop;

  public ClimberIOSparkMax(int motor1, int dioBottomLimit, int dioTopLimit) {
    motor = new SparkMax(motor1, MotorType.kBrushless);
    motorRelativeEncoder = (SparkRelativeEncoder) motor.getEncoder();
    LimitSwitchBottom = new DigitalInput(dioBottomLimit);
    LimitSwitchTop = new DigitalInput(dioTopLimit);

    SparkMaxConfig climberconfig = new SparkMaxConfig();

    climberconfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.ClimberConstants.CurrentLimit);

    climberconfig
        .encoder
        .positionConversionFactor(Constants.ClimberConstants.ClimberPositionConversionFactor)
        .velocityConversionFactor(Constants.ClimberConstants.ClimberVelocityConversionFactor);

    motor.configure(climberconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberDown = LimitSwitchBottom.get();
    inputs.climberUp = LimitSwitchTop.get();
    inputs.motorCurrent = motor.getOutputCurrent();
    inputs.motorVoltage = motor.getAppliedOutput();
    inputs.motorPosition = motorRelativeEncoder.getPosition();
  }

  public void setOutput(double speed) {
    motor.set(speed);
  }
}
