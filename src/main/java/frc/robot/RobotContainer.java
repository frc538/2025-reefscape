// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.servohub.config.ServoHubConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.climb.ClimberIO;
import frc.robot.subsystems.climb.ClimberIOSparkMax;
import frc.robot.subsystems.climb.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final ClimberSubsystem climberSubsystem;
  private final Elevator elevator;
  private final Arm arm;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController mechanismController = new CommandXboxController(1);

  // Servo Hub
  private final ServoHub servoHub = new ServoHub(3);

  LoggedNetworkNumber PDotGainNN = new LoggedNetworkNumber("/SmartDashboard/PDot Gain", 0.1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // DriveCommands.ANGLE_MAX_ACCELERATION = (Put trigger value here-->);
  // DriveCommands.ANGLE_MAX_VELOCITY = (Put trigger value here-->);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        climberSubsystem =
            new ClimberSubsystem(
                new ClimberIOSparkMax(Constants.ClimberConstants.ClimberMotorCANId, 5, 6));

        elevator =
            new Elevator(
                new ElevatorIOSparkMax(
                    Constants.ElevatorConstants.leftCanId,
                    Constants.ElevatorConstants.rightCanId,
                    Constants.ElevatorConstants.elevatorUpLimitDIOChannel,
                    Constants.ElevatorConstants.elevatorDownLimitDIOChannel));
        arm = new Arm(new ArmIOSparkMax(Constants.ArmConstants.ArmCanID));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        elevator = new Elevator(new ElevatorIOSim(0));
        arm = new Arm(new ArmIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        arm = new Arm(new ArmIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    ServoHubConfig shc = new ServoHubConfig();
    shc.channel1
        .disableBehavior(BehaviorWhenDisabled.kDoNotSupplyPower)
        .pulseRange(500, 1500, 2500);

    shc.channel3
        .disableBehavior(BehaviorWhenDisabled.kDoNotSupplyPower)
        .pulseRange(500, 1500, 2500);

    shc.channel4
        .disableBehavior(BehaviorWhenDisabled.kDoNotSupplyPower)
        .pulseRange(500, 1500, 2500);

    servoHub.configure(shc, ResetMode.kResetSafeParameters);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));
    // Lock to 0° when A button is held

    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // suggested inputs/buttons for intake controlls.
    // // coral intake trigger
    // controller.rightTrigger().onTrue(IntakeIOImplementation.coralIntake());
    // //Algae intake trigger
    // controller.leftTrigger().onTrue(IntakeIOImplementation.AlgaeIntake());
    // Reset gyro to 0° when B button is pressed

    driveController.rightBumper().whileTrue((climberSubsystem.ClimberDown()));
    driveController.leftBumper().whileTrue((climberSubsystem.ClimberUp()));

    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driveController.y().onTrue(DriveCommands.boost());

    driveController.y().onFalse(DriveCommands.boostOff());

    mechanismController.rightBumper().onTrue(elevator.PositionUp());
    mechanismController.leftBumper().onTrue(elevator.PositionDown());

    // elevator.setDefaultCommand(
    // elevator.PDotCommand(MathUtil.applyDeadband(-mechanismController.getLeftY() *
    // PDotGainNN.get(),0.1)));

    mechanismController.povUp().whileTrue(elevator.PDotCommand(0.006));
    mechanismController.povDown().whileTrue(elevator.PDotCommand(-0.006));

    arm.setDefaultCommand(arm.MoveArm(() -> mechanismController.getRightY()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
