// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
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
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.WristExtender.WristExtender;
import frc.robot.subsystems.WristExtender.WristExtenderIO;
import frc.robot.subsystems.WristExtender.WristExtenderIOServo;
import frc.robot.subsystems.climb.ClimberIO;
import frc.robot.subsystems.climb.ClimberIOSparkMax;
import frc.robot.subsystems.climb.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final WristExtender wristExtender;
  private final ClimberSubsystem climberSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController driverTwoController = new CommandXboxController(1);

  // Servo Hub
  private final ServoHub servoHub = new ServoHub(3);

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
        wristExtender =
            new WristExtender(
                new WristExtenderIOServo(
                    servoHub, ChannelId.kChannelId4, ChannelId.kChannelId3, 3, 4));

        climberSubsystem =
            new ClimberSubsystem(
                new ClimberIOSparkMax(Constants.ClimberConstants.ClimberMotorCANId, 5, 6));

        intakeSubsystem = new IntakeSubsystem(servoHub);
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
        wristExtender = new WristExtender(new WristExtenderIO() {});
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        intakeSubsystem = new IntakeSubsystem(servoHub);
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
        wristExtender = new WristExtender(new WristExtenderIO() {});
        climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
        intakeSubsystem = new IntakeSubsystem(servoHub);
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
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Start button will intake algae/shoot coral
    controller.start().onTrue(wristExtender.intakeAlgaeShootCoral());

    // Back button will intake coral/shoot algae
    controller.back().onTrue(wristExtender.intakeCoralShootAlgae());

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // suggested inputs/buttons for intake controlls.
    // // coral intake trigger
    // controller.rightTrigger().onTrue(IntakeIOImplementation.coralIntake());
    // //Algae intake trigger
    // controller.leftTrigger().onTrue(IntakeIOImplementation.AlgaeIntake());
    // Reset gyro to 0° when B button is pressed

    controller.rightBumper().whileTrue((climberSubsystem.ClimberDown()));
    controller.leftBumper().whileTrue((climberSubsystem.ClimberUp()));

    driverTwoController.axisLessThan(1, -0.5).onTrue(intakeSubsystem.feedGregory());
    driverTwoController.axisGreaterThan(1, 0.5).onTrue(intakeSubsystem.receiveCoral());

    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller.y().onTrue(DriveCommands.boost());

    controller.y().onFalse(DriveCommands.boostOff());
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
