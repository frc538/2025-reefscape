package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ClimberIOSparkMax implements ClimberIO {
    private final SparkMax motor;
    private final DigitalInput LimitSwitchBottom;
    private final DigitalInput LimitSwitchTop;
    private final SparkMaxConfig climberconfig = new SparkMaxConfig();
    public ClimberIOSparkMax(int motor1, int dioBottomLimit, int dioTopLimit) {
        motor = new SparkMax(motor1, null);
        LimitSwitchBottom = new DigitalInput(dioBottomLimit);
        LimitSwitchTop = new DigitalInput(dioTopLimit);

        climberconfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.ClimberConstants.CurrentLimit);

        climberconfig
        .encoder
        .positionConversionFactor(Constants.ModuleConstants.DrivePositionConversionFactor)
        .velocityConversionFactor(Constants.ModuleConstants.DriveVelocityConversionFactor);

        climberconfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.04, 0, 0)
        .velocityFF(1 / Constants.ModuleConstants.DriveWheelFreeSpeedMetersPerSecond)
        .outputRange(-1, 1);
    }
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.ClimberDown = LimitSwitchBottom.get();
        inputs.ClimberUp = LimitSwitchTop.get();
    }
    public  void setOutput(double speed){
        motor.set(speed);
    }
    
}
