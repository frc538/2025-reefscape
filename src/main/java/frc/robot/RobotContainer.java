// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.GyroIO;
import frc.robot.subsystems.GyroIOPigeon;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveModuleIO;
import frc.robot.subsystems.SwerveModuleIOSparkmax;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  private final SendableChooser<Command> mAutoChooser = new SendableChooser<>();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final SwerveDriveSubsystem mDriveSubsystem;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        mDriveSubsystem =
            new SwerveDriveSubsystem(
                new SwerveModuleIOSparkmax(
                    Constants.SparkMaxCANID.kFrontLeftDrive,
                    Constants.SparkMaxCANID.kFrontLeftTurn,
                    Constants.DriveConstants.kFrontLeftChassisAngularOffset),
                new SwerveModuleIOSparkmax(
                    Constants.SparkMaxCANID.kFrontRightDrive,
                    Constants.SparkMaxCANID.kFrontRightTurn,
                    Constants.DriveConstants.kFrontRightChassisAngularOffset),
                new SwerveModuleIOSparkmax(
                    Constants.SparkMaxCANID.kRearLeftDrive,
                    Constants.SparkMaxCANID.kRearLefttTurn,
                    Constants.DriveConstants.kBackLeftChassisAngularOffset),
                new SwerveModuleIOSparkmax(
                    Constants.SparkMaxCANID.kRearRightDrive,
                    Constants.SparkMaxCANID.kRearRightTurn,
                    Constants.DriveConstants.kBackRightChassisAngularOffset),
                new GyroIOPigeon(0));
        break;
      default:
        mDriveSubsystem =
            new SwerveDriveSubsystem(
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new GyroIO() {});
        break;
    }
    // Configure the trigger bindings
    configureBindings();

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      return;
    }
    // Configure AutoBuilder last
    AutoBuilder.configure(
        mDriveSubsystem::getPose, // Robot pose supplier
        mDriveSubsystem
            ::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        mDriveSubsystem::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            mDriveSubsystem.drive(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                false), // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds. Also optionally outputs individual module
        // feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for
            // holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        mDriveSubsystem // Reference to this subsystem to set requirements
        );
    // Configure the trigger bindings
    Command kLeaveAuto = new PathPlannerAuto("Leave");
    Command kCoralAuto = new PathPlannerAuto("Coral");
    mAutoChooser.setDefaultOption("No auto", null);
    mAutoChooser.addOption("Leave", kLeaveAuto);
    mAutoChooser.addOption("Coral", kCoralAuto);
    SmartDashboard.putData("Auto Selector", mAutoChooser);
  }

  private void configureBindings() {
    m_driverController.button(1).whileTrue(mDriveSubsystem.setXCommand()); // a (green) - while true
    m_driverController
        .button(2)
        .onTrue(mDriveSubsystem.toggleFieldRelativeCommand()); // b (red) button - on true
    m_driverController
        .button(7)
        .or(m_driverController.button(8))
        .onTrue(mDriveSubsystem.resetGyroCommand()); // start and back buttons - on true

    mDriveSubsystem.setDefaultCommand(
        Commands.run(
            () -> {
              Logger.recordOutput("isSetX true", false);
              double forwardSpeed = MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1);
              double leftSpeed = MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1);
              double counterclockwiseSpeed =
                  MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1);

              if (Math.abs(forwardSpeed) < 0.1) forwardSpeed = 0;
              if (Math.abs(leftSpeed) < 0.1) leftSpeed = 0;
              if (Math.abs(counterclockwiseSpeed) < 0.1) counterclockwiseSpeed = 0;

              mDriveSubsystem.drive(forwardSpeed, leftSpeed, counterclockwiseSpeed, true);
            },
            mDriveSubsystem));
  }

  public Command getAutonomousCommand() {
    return mAutoChooser.getSelected();
  }
}
