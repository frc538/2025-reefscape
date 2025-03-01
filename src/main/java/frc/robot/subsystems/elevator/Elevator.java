package frc.robot.subsystems.elevator;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase  {
    ElevatorIO io;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    LoggedNetworkNumber troughHeight = new LoggedNetworkNumber("/SmartDashboard/Trough Height",1);
    LoggedNetworkNumber coralLowHeight = new LoggedNetworkNumber("/SmartDashboard/Coral Low Height",2);
    LoggedNetworkNumber coralMedHeight = new LoggedNetworkNumber("/SmartDashboard/Coral Med Height",3);
    LoggedNetworkNumber coralHighHeight = new LoggedNetworkNumber("/SmartDashboard/Coral High Height",4);
    LoggedNetworkNumber bargeHeight = new LoggedNetworkNumber("/SmartDashboard/Barge Height",5);

    LoggedNetworkNumber arbFF = new LoggedNetworkNumber("/SmartDashboard/Arbitrary FF Gain",Constants.ElevatorConstants.arbitraryFeedForward);
    double lastArbFF = 0.0;

    public Elevator(ElevatorIO IO) {
        io = IO;
    }
    public Command Bottom() {
        return runOnce(() -> io.setReference(0));
    }
    public Command Trough() {
        return runOnce(() -> io.setReference(troughHeight.get()));
    }
    public Command CoralLow() {
        return runOnce(() -> io.setReference(coralLowHeight.get()));
    }
    public Command CoralMed() {
        return runOnce(() -> io.setReference(coralMedHeight.get()));
    }
    public Command CoralHigh() {
        return runOnce(() -> io.setReference(coralHighHeight.get()));
    }
    public Command Barge() {
        return runOnce(() -> io.setReference(bargeHeight.get()));
    }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs); 
        Logger.recordOutput("Elevator/Trough Height",troughHeight.get());

        if (arbFF.get() != lastArbFF) {
            io.setConfiguration(arbFF.get());
            lastArbFF = arbFF.get();
        }

        
    }
}