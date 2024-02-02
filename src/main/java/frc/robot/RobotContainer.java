// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autonomous.LeaveAuto;
import frc.robot.commands.autonomous.SimpleAuto;
import frc.robot.subsystems.WaggleSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(
    DriveSubsystem.initializeHardware(),
    Constants.Drive.DRIVE_ROTATE_PID,
    Constants.Drive.DRIVE_CONTROL_CENTRICITY,
    Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_SCALAR,
    Constants.HID.CONTROLLER_DEADBAND,
    Constants.Drive.DRIVE_LOOKAHEAD
  );
  private static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem(
    ShooterSubsystem.initializeHardware(),
    Constants.Shooter.FLYWHEEL_CONFIG,
    Constants.Shooter.ANGLE_CONFIG,
    Constants.Shooter.ANGLE_FF,
    Constants.Shooter.ANGLE_MOTION_CONSTRAINT,
    Constants.Shooter.FLYWHEEL_DIAMETER,
    Constants.Shooter.SHOOTER_MAP,
    DRIVE_SUBSYSTEM::getPose,
    () -> speakerSupplier()
  );

  private static final WaggleSubsystem WIGGLE_STICK = new WaggleSubsystem(Constants.WiggleStick.WIGGLE_STICK_CONFIG, Constants.WiggleStick.CONSTRAINTS);

  private static final VisionSubsystem VISION_SUBSYSTEM = VisionSubsystem.getInstance();

  private static final CommandXboxController PRIMARY_CONTROLLER = new CommandXboxController(Constants.HID.PRIMARY_CONTROLLER_PORT);

  private static SendableChooser<Command> m_automodeChooser = new SendableChooser<>();

  public RobotContainer() {
    // Set drive command
    DRIVE_SUBSYSTEM.setDefaultCommand(
      DRIVE_SUBSYSTEM.driveCommand(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getLeftX(),
        () -> PRIMARY_CONTROLLER.getRightX()
      )
    );

    // Setup AutoBuilder
    DRIVE_SUBSYSTEM.configureAutoBuilder();

    VISION_SUBSYSTEM.setPoseSupplier(() -> DRIVE_SUBSYSTEM.getPose());

    // Bind buttons and triggers
    configureBindings();

    // Configure ShuffleBoard
    defaultShuffleboardTab();
  }

  private void configureBindings() {
    // Start button - toggle traction control
    PRIMARY_CONTROLLER.start().onTrue(DRIVE_SUBSYSTEM.toggleTractionControlCommand());

    // Y button - aim at speaker
    PRIMARY_CONTROLLER.y().whileTrue(shootCommand());

    // Right bumper button - go to amp
    PRIMARY_CONTROLLER.rightBumper().whileTrue(
      DRIVE_SUBSYSTEM.goToPoseCommand(
        Constants.Field.AMP,
        SHOOTER_SUBSYSTEM.prepareForAmpCommand(),
        Commands.none()
      )
    );

    // A button - go to source
    PRIMARY_CONTROLLER.a().whileTrue(DRIVE_SUBSYSTEM.goToPoseCommand(Constants.Field.SOURCE));

    // B button - aim at game object
    PRIMARY_CONTROLLER.b().whileTrue(
      DRIVE_SUBSYSTEM.aimAtPointCommand(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getLeftX(),
        () -> VISION_SUBSYSTEM.getObjectTranslation(),
        false,
        false
      )
    );

    PRIMARY_CONTROLLER.povLeft().onTrue(WIGGLE_STICK.setPositionCommand(0.0));
    PRIMARY_CONTROLLER.povRight().onTrue(WIGGLE_STICK.setPositionCommand(15.0));
  }

  /**
   * Compose command to shoot note
   * @return Command that will automatically aim and shoot note
   */
  private Command shootCommand() {
    return Commands.parallel(
      DRIVE_SUBSYSTEM.aimAtPointCommand(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getLeftX(),
        () -> speakerSupplier().getSecond(),
        true,
        true
      ),
      SHOOTER_SUBSYSTEM.shootCommand(() -> DRIVE_SUBSYSTEM.isAimed())
    );
  }

  /**
   * Get correct speaker for current alliance
   * @return Location of appropriate speaker
   */
  private static Pair<Integer,Translation2d> speakerSupplier() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
      ? Constants.Field.BLUE_SPEAKER
      : Constants.Field.RED_SPEAKER;
  }

  /**
   * Add auto modes to chooser
   */
  private void autoModeChooser() {
    m_automodeChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
    m_automodeChooser.addOption("Simple", new SimpleAuto(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Leave", new LeaveAuto(DRIVE_SUBSYSTEM));
  }

  /**
   * Run simlation related methods
   */
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  /**
   * Get currently selected autonomous command
   *
   * @return Autonomous command
   */
  public Command getAutonomousCommand() {
    return m_automodeChooser.getSelected();
  }

  /**
   * Configure default Shuffleboard tab
   */
  public void defaultShuffleboardTab() {
    Shuffleboard.selectTab(Constants.SmartDashboard.SMARTDASHBOARD_DEFAULT_TAB);
    autoModeChooser();
    SmartDashboard.putData(Constants.SmartDashboard.SMARTDASHBOARD_AUTO_MODE, m_automodeChooser);
  }
}
