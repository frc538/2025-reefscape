// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.servohub.config.ServoHubConfig;
import edu.wpi.first.math.MathUtil;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOServo;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightIO;
import frc.robot.subsystems.limelight.LimelightIOImplementation;
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
  private final Intake intake;
  private final Limelight limelight;

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

        intake =
            new Intake(
                new IntakeIOServo(
                    servoHub,
                    Constants.IntakeConstants.leftChannel,
                    Constants.IntakeConstants.rightChannel));

        elevator =
            new Elevator(
                new ElevatorIOSparkMax(
                    Constants.ElevatorConstants.leftCanId,
                    Constants.ElevatorConstants.rightCanId,
                    Constants.ElevatorConstants.elevatorUpLimitDIOChannel,
                    Constants.ElevatorConstants.elevatorDownLimitDIOChannel));
        arm = new Arm(new ArmIOSparkMax(Constants.ArmConstants.ArmCanID));
        LimelightIOImplementation[] Limelights = new LimelightIOImplementation[2];

        Limelights[0] =
            new LimelightIOImplementation(Constants.LimeLightConstants.limelightOneName);
        Limelights[1] =
            new LimelightIOImplementation(Constants.LimeLightConstants.limelightTwoName);

        limelight = new Limelight(drive, Limelights);
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
        intake = new Intake(new IntakeIO() {});
        LimelightIO[] LimelightsSim = new LimelightIO[2];

        LimelightsSim[0] = new LimelightIO() {};
        LimelightsSim[1] = new LimelightIO() {};

        limelight = new Limelight(drive, LimelightsSim);
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
        intake = new Intake(new IntakeIO() {});
        arm = new Arm(new ArmIO() {});

        LimelightIO[] LimelightsReplay = new LimelightIO[2];

        LimelightsReplay[0] = new LimelightIO() {};
        LimelightsReplay[1] = new LimelightIO() {};

        limelight = new Limelight(drive, LimelightsReplay);

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

    shc.channel0
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
    // DRIVE CONTROLLER COMMANDS

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));
    // Lock to 0° when A button is held

    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

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
    driveController.rightBumper().whileTrue((climberSubsystem.ClimberDown()));
    driveController.leftBumper().whileTrue((climberSubsystem.ClimberUp()));
    climberSubsystem.setDefaultCommand(
        Commands.run(
            () -> {
              climberSubsystem.ClimberSpeed(
                  () -> {
                    return driveController.getRightTriggerAxis()
                        - driveController.getLeftTriggerAxis();
                  });
            },
            climberSubsystem));
    driveController.rightStick().onTrue(DriveCommands.boost());
    driveController.rightStick().onFalse(DriveCommands.boostOff());

    // MECHANISM CONTROLLER COMMANDS

    mechanismController.rightBumper().onTrue(elevator.PositionUp());
    mechanismController.leftBumper().onTrue(elevator.PositionDown());
    mechanismController.povUp().whileTrue(elevator.PDotCommand(0.006));
    mechanismController.povDown().whileTrue(elevator.PDotCommand(-0.006));
    // arm.setDefaultCommand(arm.MoveArm(() -> mechanismController.getRightY()));\
    arm.setDefaultCommand(
        arm.RateCommand(() -> MathUtil.applyDeadband(mechanismController.getRightY(), 0.1)));
    mechanismController.a().whileTrue(intake.intakeIn());
    mechanismController.y().whileTrue(intake.intakeOut());
    mechanismController.x().onTrue(arm.runArmToggle());
    mechanismController.button(8).whileTrue(intake.intakeReset());
    // elevator.setDefaultCommand(
    // elevator.PDotCommand(MathUtil.applyDeadband(-mechanismController.getLeftY() *
    // PDotGainNN.get(),0.1)));
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
