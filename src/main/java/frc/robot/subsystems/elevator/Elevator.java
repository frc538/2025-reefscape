package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    ElevatorIO io;

    private int positionTarget = 0;
    private int positionMax = 5;
    private int positionMin = 0;

    private double buttonPositionCommand = 0.0;
    private double PDotPositionCommand = 0.0;
    private double PDotRate = 0.0;
    private boolean UseButtonState = true;

    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    LoggedNetworkNumber troughHeight = new LoggedNetworkNumber("/SmartDashboard/Trough Height", 0);
    LoggedNetworkNumber coralLowHeight = new LoggedNetworkNumber("/SmartDashboard/Coral Low Height", .2);
    LoggedNetworkNumber coralMedHeight = new LoggedNetworkNumber("/SmartDashboard/Coral Med Height", .4);
    LoggedNetworkNumber coralHighHeight = new LoggedNetworkNumber("/SmartDashboard/Coral High Height", .6);
    LoggedNetworkNumber bargeHeight = new LoggedNetworkNumber("/SmartDashboard/Barge Height", .8);

    LoggedNetworkNumber arbFF = new LoggedNetworkNumber("/SmartDashboard/Arbitrary FF Gain", 0.5);
    
    public Elevator(ElevatorIO IO) {
        io = IO;
    }

    public Command PositionUp() {
        if (positionTarget < positionMax) {
            positionTarget = positionTarget + 1;
        }
        return GoToPosition();
    }

    public Command PositionDown() {
        if (positionTarget > positionMin) {
            positionTarget = positionTarget - 1;
        }
        return GoToPosition();
    }

    private Command GoToPosition() {
        switch (positionTarget) {
            case 0:
                return Bottom();
            case 1:
                return Trough();
            case 2:
                return CoralLow();
            case 3:
                return CoralMed();
            case 4:
                return CoralHigh();
            case 5:
                return Barge();

            default:
                return runOnce(() -> positionTarget = 0);
        }
    }

    public Command Bottom() {
        return runOnce(() -> PositionCommand(0));
    }

    public Command Trough() {
        return runOnce(() -> PositionCommand(troughHeight.get()));
    }

    public Command CoralLow() {
        return runOnce(() -> PositionCommand(coralLowHeight.get()));
    }

    public Command CoralMed() {
        return runOnce(() -> PositionCommand(coralMedHeight.get()));
    }

    public Command CoralHigh() {
        return runOnce(() -> PositionCommand(coralHighHeight.get()));
    }

    public Command Barge() {
        return runOnce(() -> PositionCommand(bargeHeight.get()));
    }

    private void PositionCommand(double position) {
        io.setReference(position);
        buttonPositionCommand = position;
        UseButtonState = true;
        PDotPositionCommand = buttonPositionCommand;
    }

    public Command PDotCommand(double rate) {
        return run(() -> doRate(rate));
    }

    private void doRate(double rate) {
        if (rate != 0) {
            PDotRate = rate;
            UseButtonState = false;
            PDotPositionCommand = PDotPositionCommand + PDotRate;
            if (PDotPositionCommand < 0) {
                PDotPositionCommand = 0;
            }
            io.setReference(PDotPositionCommand);
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator/Trough Height", troughHeight.get());
        if (UseButtonState) {
            Logger.recordOutput("Elevator/Command Source", "Button");
        } else {
            Logger.recordOutput("Elevator/Command Source", "Stick");

        }
        
        io.setArbFF(arbFF.get());
        
        if (inputs.atBottom) {
            io.resetEncoders();
        }

        io.commandMotor();

        Logger.recordOutput("Elevator/Button Command", buttonPositionCommand);
        Logger.recordOutput("Elevator/PDot Command", PDotPositionCommand);
    }
}