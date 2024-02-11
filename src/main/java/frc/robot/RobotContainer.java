// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autonomous.LeaveAuto;
import frc.robot.commands.autonomous.SimpleAuto;
import frc.robot.commands.autonomous.TestAuto;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
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
  private static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem(
    IntakeSubsystem.initializeHardware(),
    Constants.Intake.ROLLER_VELOCITY
  );
  private static final ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem(
    ClimberSubsystem.initializeHardware(),
    Constants.Climber.CLIMBER_VELOCITY
  );

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

    // Right trigger button - aim and shoot at speaker, shooting regardless if speaker tag is visible if center of DPAD is pressed
    PRIMARY_CONTROLLER.rightTrigger().whileTrue(
      shootCommand(PRIMARY_CONTROLLER.povCenter().getAsBoolean())
    );

    // Left trigger button - aim at game piece and intake
    PRIMARY_CONTROLLER.leftTrigger().whileTrue(intakeCommand());

    // Left bumper button - outtake game piece
    PRIMARY_CONTROLLER.leftBumper().whileTrue(outtakeCommand());

    // A button - go to amp
    PRIMARY_CONTROLLER.a().whileTrue(
      DRIVE_SUBSYSTEM.goToPoseCommand(
        Constants.Field.AMP,
        SHOOTER_SUBSYSTEM.prepareForAmpCommand(),
        Commands.none()
      )
    );

    // B button - go to source
    PRIMARY_CONTROLLER.b().whileTrue(DRIVE_SUBSYSTEM.goToPoseCommand(Constants.Field.SOURCE));

    // X button - Manually aim the shooter at a desired flywheel speed and angle retrieved from the SmartDashboard
    PRIMARY_CONTROLLER.x().whileTrue(SHOOTER_SUBSYSTEM.shootManualCommand(() -> dashboardStateSupplier()));
  }

  /**
   * Compose command to control controller rumble.
   * <ul>
   * <li> If the vision subsystem detects a game piece, the left side of the controller will rumble
   * <li> If the intake has a game piece inside, the right side of the controller will rumble
   * <li> Otherwise, no rumble :(
   * </ul>
   * @return Command that will automatically make the controller rumble based on the above conditions
   */
  private Command rumbleCommand() {
    return Commands.run(() -> {
      if (VISION_SUBSYSTEM.getObjectLocation().isPresent())
        PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
      else PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
      if (SHOOTER_SUBSYSTEM.isObjectPresent())
        PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kRightRumble, 1.0);
      else PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kRightRumble, 0.0);
    }).finallyDo(() -> PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kBothRumble, 0.0));
  }

  /**
   * Compose command to intake a note
   * @return Command that will automatically intake a note and prepare it for feeding inside the shooter motor
   */
  private Command intakeCommand() {
    return Commands.parallel(
      rumbleCommand(),
      aimAtObject(),
      INTAKE_SUBSYSTEM.intakeCommand(),
      SHOOTER_SUBSYSTEM.intakeCommand()
    );
  }

  /**
   * Compose command to outtake a note
   * @return Command that will spit out a note from ground intake
   */
  private Command outtakeCommand() {
    return Commands.parallel(
      rumbleCommand(),
      INTAKE_SUBSYSTEM.outtakeCommand(),
      SHOOTER_SUBSYSTEM.outtakeCommand()
    );
  }

 /**
   * Compose command to shoot note
   * @param ignoreTarget Shoot regardless if speaker tag is visible
   * @return Command that will automatically aim and shoot note
   */
  private Command shootCommand(boolean ignoreTarget) {
    return Commands.parallel(
      DRIVE_SUBSYSTEM.aimAtPointCommand(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getLeftX(),
        () -> PRIMARY_CONTROLLER.getRightX(),
        () -> speakerSupplier().getSecond(),
        true,
        true
      ),
      SHOOTER_SUBSYSTEM.shootCommand(() -> DRIVE_SUBSYSTEM.isAimed(), ignoreTarget)
    );
  }

  /**
   * Compose command to feed a note through the robot
   * @return Command that will run the intake, indexer, and flywheels at the same time with limit switches disabled
   */
  private Command feedThroughCommand() {
    return Commands.parallel(
      rumbleCommand(),
      INTAKE_SUBSYSTEM.intakeCommand(),
      SHOOTER_SUBSYSTEM.feedCommand()
    );
  }

  /**
   * Command to aim at detected game object automatically, driving normally if none is detected
   * @return Command to aim at object
   */
  private Command aimAtObject() {
    return DRIVE_SUBSYSTEM.aimAtPointCommand(
      () -> PRIMARY_CONTROLLER.getLeftY(),
      () -> PRIMARY_CONTROLLER.getLeftX(),
      () -> PRIMARY_CONTROLLER.getRightX(),
      () -> {
        return VISION_SUBSYSTEM.getObjectLocation().isPresent()
                ? VISION_SUBSYSTEM.getObjectLocation().get()
                : null;
      },
      false,
      false
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
   * Manually retrieve a desired shooter state from the dashboard
   * @return Shooter state with the desired speed and angle
   */
  private ShooterSubsystem.State dashboardStateSupplier() {
    return new ShooterSubsystem.State(
      Units.MetersPerSecond.of(SmartDashboard.getNumber(Constants.SmartDashboard.SMARTDASHBOARD_SHOOTER_SPEED, 0.0)),
      Units.Degrees.of(SmartDashboard.getNumber(Constants.SmartDashboard.SMARTDASHBOARD_SHOOTER_ANGLE, 0.0))
    );
  }

  /**
   * Add auto modes to chooser
   */
  private void autoModeChooser() {
    m_automodeChooser.setDefaultOption("Do nothing", Commands.none());
    m_automodeChooser.addOption("Simple", new SimpleAuto(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Leave", new LeaveAuto(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("Test", new TestAuto(DRIVE_SUBSYSTEM));
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
    SmartDashboard.putNumber(Constants.SmartDashboard.SMARTDASHBOARD_SHOOTER_SPEED, 0);
    SmartDashboard.putNumber(Constants.SmartDashboard.SMARTDASHBOARD_SHOOTER_ANGLE, 0);
  }
}
