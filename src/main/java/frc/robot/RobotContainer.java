// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveModuleIO;
import frc.robot.subsystems.SwerveModuleIOSparkmax;

import java.util.concurrent.DelayQueue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final SendableChooser<Command> mAutoChooser = new SendableChooser<>();

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final SwerveDriveSubsystem mDriveSubsystem;
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        mDriveSubsystem = new SwerveDriveSubsystem(
            new SwerveModuleIOSparkmax(Constants.SparkMaxCANID.kFrontLeftDrive,
                Constants.SparkMaxCANID.kFrontLeftTurn,
                Constants.DriveConstants.kFrontLeftChassisAngularOffset),
            new SwerveModuleIOSparkmax(Constants.SparkMaxCANID.kFrontRightDrive,
                Constants.SparkMaxCANID.kFrontRightTurn,
                Constants.DriveConstants.kFrontRightChassisAngularOffset),
            new SwerveModuleIOSparkmax(Constants.SparkMaxCANID.kRearLeftDrive,
                Constants.SparkMaxCANID.kRearLefttTurn,
                Constants.DriveConstants.kBackLeftChassisAngularOffset),
            new SwerveModuleIOSparkmax(Constants.SparkMaxCANID.kRearRightDrive,
                Constants.SparkMaxCANID.kRearRightTurn,
                Constants.DriveConstants.kBackRightChassisAngularOffset),
            0);
        break;
      default:
        mDriveSubsystem = new SwerveDriveSubsystem(
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            0);
        break;
    }
    // Configure the trigger bindings
    configureBindings();

    mAutoChooser.setDefaultOption("No auto", null);
    Command leave = Commands
      .run(() -> mDriveSubsystem.drive(0.5, 0, 0, true), mDriveSubsystem)
      .withTimeout(3);
    Command coral = Commands
      .run(() -> mDriveSubsystem.drive(0.5, 0, 0, true), mDriveSubsystem)
      .withTimeout(3);
    mAutoChooser.addOption("Leave", leave);
    mAutoChooser.addOption("Coral", coral);

    SmartDashboard.putData("Auto Selector", mAutoChooser);
  }

  private void configureBindings() {
    m_driverController.button(1).whileTrue(mDriveSubsystem.setXCommand()); //a (green) - while true
    m_driverController.button(2).onTrue(mDriveSubsystem.toggleFieldRelative()); //b (red) button - on true
    m_driverController.button(7).or(m_driverController.button(8)).onTrue(mDriveSubsystem.resetGyroCommand()); //start and back buttons - on true

    mDriveSubsystem.setDefaultCommand(Commands.run(() -> {
      double forwardSpeed = MathUtil.applyDeadband(-m_driverController.getLeftY(),0.1);
      double leftSpeed = MathUtil.applyDeadband(-m_driverController.getLeftX(),0.1);
      double counterclockwiseSpeed = MathUtil.applyDeadband(-m_driverController.getRightX(),0.1);

      if (Math.abs(forwardSpeed) < 0.1) forwardSpeed = 0;
      if (Math.abs(leftSpeed) < 0.1) leftSpeed = 0;
      if (Math.abs(counterclockwiseSpeed) < 0.1) counterclockwiseSpeed = 0;

      mDriveSubsystem.drive(forwardSpeed, leftSpeed, counterclockwiseSpeed, true);
    }, mDriveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
