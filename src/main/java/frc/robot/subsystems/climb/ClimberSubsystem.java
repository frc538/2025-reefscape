package frc.robot.subsystems.climb;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    public ClimberSubsystem(ClimberIO IO){
        io = IO;

    }
    public Command ClimberUp(){
        return Commands.run(()->io.setOutput(-0.1),this);
    }
    public Command ClimberDown(){
        return Commands.run(()->io.setOutput(0.1),this);
    }
    public Command Stop(){
        return Commands.runOnce(()->io.setOutput(0),this);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("climber subsystem", inputs);
    }
}
