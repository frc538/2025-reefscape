package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimberIO;
import frc.robot.subsystems.climb.ClimberIOInputsAutoLogged;

public class Arm extends SubsystemBase {
    ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO IO) {
        io = IO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("climber subsystem", inputs);
    }
}
