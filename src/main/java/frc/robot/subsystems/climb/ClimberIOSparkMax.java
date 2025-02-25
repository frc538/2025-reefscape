package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;

import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIOSparkMax implements ClimberIO {
    public final SparkMax motorLeft;
    public final SparkMax motorRight;
    public final DigitalInput LimitSwitchBottom;
    public final DigitalInput LimitSwitchTop;
    public ClimberIOSparkMax(int motor1, int motor2, int dioBottomLimit, int dioTopLimit) {
        motorLeft = new SparkMax(motor1, null);
        motorRight = new SparkMax(motor2, null);
        LimitSwitchBottom = new DigitalInput(dioBottomLimit);
        LimitSwitchTop = new DigitalInput(dioTopLimit);
    }
}
