package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimberSubsystem {
    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    public ClimberSubsystem(ClimberIO IO){
        io = IO;

    }
    public Command ClimberUp(){
        return Commands.run(()->io.setOutput(10));
    }
    public Command ClimberDown(){
        return Commands.run(()->io.setOutput(-10));
    }
}
