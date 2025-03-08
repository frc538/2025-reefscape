package frc.robot.subsystems.elevator;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase  {
    ElevatorIO io;
    
    private int positionTarget = 0;
    private int positionMax = 6;

    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    LoggedNetworkNumber troughHeight = new LoggedNetworkNumber("/SmartDashboard/Trough Height",0);
    LoggedNetworkNumber coralLowHeight = new LoggedNetworkNumber("/SmartDashboard/Coral Low Height",.2);
    LoggedNetworkNumber coralMedHeight = new LoggedNetworkNumber("/SmartDashboard/Coral Med Height",.4);
    LoggedNetworkNumber coralHighHeight = new LoggedNetworkNumber("/SmartDashboard/Coral High Height",.6);
    LoggedNetworkNumber bargeHeight = new LoggedNetworkNumber("/SmartDashboard/Barge Height",.8);

    LoggedNetworkNumber arbFF = new LoggedNetworkNumber("/SmartDashboard/Arbitrary FF Gain",1.32);
    double lastArbFF = 0.0;

    public Elevator(ElevatorIO IO) {
        io = IO;
    }
    public Command PositionUp() {
        if (positionTarget < positionMax) {
            positionTarget = positionTarget + 1;
        }
    }
    public Command PositionDown() {
        if (positionTarget < positionMax) {
            positionTarget = positionTarget + 1;
        }
    }
    private Command GoToPosition() {
        switch(positionTarget) {
            case 0: return Bottom();
            case 1: return Trough();
            case 2: return CoralLow();
            case 3: return CoralMed();
            case 4: return CoralHigh();
            case 5: return Barge();
        }
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
            io.setArbFF(arbFF.get());
            lastArbFF = arbFF.get();
        }

        io.commandMotor();
    }
}