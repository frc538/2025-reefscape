package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ClimberIOSparkMax implements ClimberIO {
    private final SparkMax motor;
    private SparkRelativeEncoder motorRelativeEncoder;
    private final DigitalInput LimitSwitchBottom;
    private final DigitalInput LimitSwitchTop;
    private final SparkMaxConfig climberconfig = new SparkMaxConfig();
    public ClimberIOSparkMax(int motor1, int dioBottomLimit, int dioTopLimit) {
        motor = new SparkMax(motor1, null);
        motorRelativeEncoder = (SparkRelativeEncoder) motor.getEncoder();
        LimitSwitchBottom = new DigitalInput(dioBottomLimit);
        LimitSwitchTop = new DigitalInput(dioTopLimit);

        climberconfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.ClimberConstants.CurrentLimit);

        climberconfig
        .encoder
        .positionConversionFactor(Constants.ClimberConstants.ClimberPositionConversionFactor)
        .velocityConversionFactor(Constants.ClimberConstants.ClimberVelocityConversionFactor);

    }
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberDown = LimitSwitchBottom.get();
        inputs.climberUp = LimitSwitchTop.get();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorVoltage = motor.getAppliedOutput();
        inputs.motorPosition = motorRelativeEncoder.getPosition();
    }
    public  void setOutput(double speed){
        motor.set(speed);
    }
    
}
