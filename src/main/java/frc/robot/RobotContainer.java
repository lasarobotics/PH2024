// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.UUID;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.autonomous.SimpleAuto;
import frc.robot.subsystems.drive.AutoTrajectory;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.State;
import frc.robot.subsystems.vision.VisionSubsystem;

@SuppressWarnings("unused")
public class RobotContainer {
  private static final DriveSubsystem
  DRIVE_SUBSYSTEM = new DriveSubsystem(
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
    Constants.Shooter.ANGLE_MOTION_CONSTRAINT,
    Constants.Shooter.TOP_FLYWHEEL_DIAMETER,
    Constants.Shooter.BOTTOM_FLYWHEEL_DIAMETER,
    Constants.Shooter.SHOOTER_MAP,
    DRIVE_SUBSYSTEM::getPose,
    () -> speakerSupplier()
  );
  private static final IntakeSubsystem INTAKE_SUBSYSTEM = IntakeSubsystem.getInstance(IntakeSubsystem.initializeHardware());
  private static final VisionSubsystem VISION_SUBSYSTEM = VisionSubsystem.getInstance();

  private static final CommandXboxController PRIMARY_CONTROLLER = new CommandXboxController(Constants.HID.PRIMARY_CONTROLLER_PORT);

  private static SendableChooser<Command> m_automodeChooser = new SendableChooser<>();

  public RobotContainer() {
    // Silence warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    // Set drive command
    DRIVE_SUBSYSTEM.setDefaultCommand(
      DRIVE_SUBSYSTEM.driveCommand(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getLeftX(),
        () -> PRIMARY_CONTROLLER.getRightX()
      )
    );

    // Configure auto builder
    DRIVE_SUBSYSTEM.configureAutoBuilder();

    VISION_SUBSYSTEM.setPoseSupplier(() -> DRIVE_SUBSYSTEM.getPose());

    // Register named commands
    NamedCommands.registerCommand(Constants.NamedCommands.INTAKE_COMMAND_NAME, autoIntakeCommand().withTimeout(7));
    NamedCommands.registerCommand(Constants.NamedCommands.PRELOAD_COMMAND_NAME, SHOOTER_SUBSYSTEM.shootSpeakerCommand().withTimeout(1.1));
    NamedCommands.registerCommand(Constants.NamedCommands.SHOOT_COMMAND_NAME, SHOOTER_SUBSYSTEM.shootSpeakerCommand().withTimeout(0.9));
    NamedCommands.registerCommand(Constants.NamedCommands.SPINUP_COMMAND_NAME, SHOOTER_SUBSYSTEM.spinupCommand());
    NamedCommands.registerCommand(Constants.NamedCommands.FEEDTHROUGH_COMMAND_NAME, feedThroughCommand().withTimeout(2));
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_SHOOT_COMMAND_NAME, shootCommand().withTimeout(0.9));
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_SHOOT_LONG_COMMAND_NAME, shootCommand().withTimeout(2.0));
    NamedCommands.registerCommand(Constants.NamedCommands.AUTO_INTAKE_COMMAND_NAME, aimAndIntakeObjectCommand());

    // Bind buttons and triggers
    configureBindings();

    // Configure ShuffleBoard
    defaultShuffleboardTab();
  }

  private void configureBindings() {
    // Start button - toggle traction control
    PRIMARY_CONTROLLER.start().onTrue(DRIVE_SUBSYSTEM.toggleTractionControlCommand());

    // Back button - toggles centricity between robot and field centric
    PRIMARY_CONTROLLER.back().onTrue(DRIVE_SUBSYSTEM.toggleCentricityCommand());

    // Right trigger button - aim and shoot at speaker, shooting only if speaker tag is visible and robot is in range
    // Click DPAD down to override and shoot now
    PRIMARY_CONTROLLER.rightTrigger().whileTrue(shootCommand(() -> PRIMARY_CONTROLLER.b().getAsBoolean()));

    // Right bumper button - amp score, also use for outtake
    PRIMARY_CONTROLLER.rightBumper().whileTrue(SHOOTER_SUBSYSTEM.scoreAmpCommand());

    // Left trigger button - intake game piece
    //PRIMARY_CONTROLLER.leftTrigger().whileTrue(intakeCommand());


    // Left bumper button - intake game piece from source
    PRIMARY_CONTROLLER.leftBumper().whileTrue(sourceIntakeCommand());

    // A button - go to amp and score
    // PRIMARY_CONTROLLER.a().whileTrue(
      // DRIVE_SUBSYSTEM.goToPoseCommand(
        // Constants.Field.AMP,
        // SHOOTER_SUBSYSTEM.prepareForAmpCommand(),
        // SHOOTER_SUBSYSTEM.scoreAmpCommand()
      // )
    // );

    // B button - go to source and intake game piece
    // PRIMARY_CONTROLLER.b().whileTrue(
    //   DRIVE_SUBSYSTEM.goToPoseCommand(
    //     Constants.Field.SOURCE,
    //     SHOOTER_SUBSYSTEM.sourceIntakeCommand(),
    //     SHOOTER_SUBSYSTEM.sourceIntakeCommand()
    //   )
    // );

    // Right stick click - snap robot to the nearest cardinal direction
    PRIMARY_CONTROLLER.rightStick().whileTrue(
      DRIVE_SUBSYSTEM.snapToImportantDirectionCommand(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getLeftX()
      )
    );

    // B Button - automatically aim at object
    // PRIMARY_CONTROLLER.b().whileTrue(aimAtObject());

    // A Button - shoot note into speaker from podium
    PRIMARY_CONTROLLER.a().whileTrue(SHOOTER_SUBSYSTEM.shootPodiumCommand());

    // B Button - pass note
    //PRIMARY_CONTROLLER.b().whileTrue(SHOOTER_SUBSYSTEM.passCommand());
    INTAKE_SUBSYSTEM.bindIntakeButton(PRIMARY_CONTROLLER.b());

    // X button - shoot note into speaker from against the subwoofer
    PRIMARY_CONTROLLER.x().whileTrue(SHOOTER_SUBSYSTEM.shootSpeakerCommand());

    // Y button - spit out note
    //PRIMARY_CONTROLLER.y().whileTrue(outtakeCommand());
    INTAKE_SUBSYSTEM.bindOuttakeButton(PRIMARY_CONTROLLER.y());

    // DPAD up - shoot manual
    PRIMARY_CONTROLLER.povUp().whileTrue(SHOOTER_SUBSYSTEM.shootManualCommand(() -> dashboardStateSupplier()));

    // DPAD right - feed a note through
    PRIMARY_CONTROLLER.povRight().whileTrue(feedThroughCommand());

    // DPAD down - auto defense
    PRIMARY_CONTROLLER.povDown().whileTrue(DRIVE_SUBSYSTEM.autoDefenseCommand(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getLeftX(),
        () -> PRIMARY_CONTROLLER.getRightX()
    ));

    PRIMARY_CONTROLLER.povLeft().whileTrue(aimAndIntakeObjectCommand());
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
      if (SHOOTER_SUBSYSTEM.isObjectPresent())
        PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kBothRumble, 1.0);
      else PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }).finallyDo(() -> PRIMARY_CONTROLLER.getHID().setRumble(RumbleType.kBothRumble, 0.0));
  }

  /**
   * Compose command to intake a note and rumble controller appropriately
   * @return Command that will automatically intake a note and prepare it for feeding inside the shooter motor
   */
  private Command intakeCommand() {
    return Commands.parallel(
      rumbleCommand(),
      //INTAKE_SUBSYSTEM.intakeCommand(),
      SHOOTER_SUBSYSTEM.intakeCommand()
    );
  }

  /**
   * Compose command to intake a note via the source and rumble controller appropriately
   * @return Command that will automatically intake a note from source via the shooter flywheels
   */
  private Command sourceIntakeCommand() {
    return Commands.parallel(
      rumbleCommand(),
      SHOOTER_SUBSYSTEM.sourceIntakeCommand()
    );
  }

  /**
   * Intake until an object is present in autonomous
   * @return Command to intake until an object is present
   */
  private Command autoIntakeCommand() {
    return Commands.parallel(
      //INTAKE_SUBSYSTEM.intakeCommand(),
      SHOOTER_SUBSYSTEM.intakeCommand()
    ).until(() -> SHOOTER_SUBSYSTEM.isObjectPresent());
  }

  /**
   * Compose command to outtake a note
   * @return Command that will spit out a note from ground intake
   */
  private Command outtakeCommand() {
    return Commands.parallel(
      rumbleCommand(),
      //INTAKE_SUBSYSTEM.outtakeCommand(),
      SHOOTER_SUBSYSTEM.outtakeCommand()
    );
  }

  /**
   * Compose command to shoot note
   * @param override Shoot even if target tag is not visible
   * @return Command that will automatically aim and shoot note
   */
  private Command shootCommand(BooleanSupplier override) {
    return Commands.parallel(
      DRIVE_SUBSYSTEM.aimAtPointCommand(
        () -> PRIMARY_CONTROLLER.getLeftY(),
        () -> PRIMARY_CONTROLLER.getLeftX(),
        () -> PRIMARY_CONTROLLER.getRightX(),
        () -> speakerSupplier().pose.getTranslation().toTranslation2d(),
        true,
        false
      ),
      SHOOTER_SUBSYSTEM.shootCommand(() -> DRIVE_SUBSYSTEM.isAimed(), override)
    );
  }

  /**
   * Compose command to shoot note, checking if tag is visible and robot is in range
   * @return Command that will automatically aim and shoot note
   */
  private Command shootCommand() {
    return shootCommand(() -> false);
  }

  /**
   * Compose command to feed a note through the robot
   * @return Command to feed note through the robot
   */
  private Command feedThroughCommand() {
    return Commands.parallel(
      rumbleCommand(),
      //INTAKE_SUBSYSTEM.intakeCommand(),
      SHOOTER_SUBSYSTEM.feedThroughCommand(() -> DRIVE_SUBSYSTEM.isAimed())
    );
  }

  /**
   * Command to aim at detected game object automatically, driving normally if none is detected
   * @return Command to aim at object
   */
  // private Command aimAtObject() {
  //   return DRIVE_SUBSYSTEM.aimAtPointRobotCentric(
  //     () -> PRIMARY_CONTROLLER.getLeftY(),
  //     () -> PRIMARY_CONTROLLER.getLeftX(),
  //     () -> PRIMARY_CONTROLLER.getRightX(),
  //     () -> {
  //       return VISION_SUBSYSTEM.getObjectLocation().isPresent()
  //               ? VISION_SUBSYSTEM.getObjectLocation().get()
  //               : null;
  //     },
  //     false,
  //     false
  //   );
  // }

   /**
    * Automatically aim robot heading at object, drive, and intake a game object
    * @return Command to aim robot at object, drive, and intake a game object
    */
   private Command aimAndIntakeObjectCommand() {
    //  return Commands.sequence(
    //   DRIVE_SUBSYSTEM.driveCommand(() -> 0, () -> 0, () -> 0).withTimeout(0.1),
    //   DRIVE_SUBSYSTEM.aimAtPointCommand(
    //       () -> VISION_SUBSYSTEM.getObjectLocation().isPresent() ? PRIMARY_CONTROLLER.getLeftY() : 0,
    //       () -> VISION_SUBSYSTEM.getObjectLocation().isPresent() ? PRIMARY_CONTROLLER.getLeftX() : 0,
    //       () -> VISION_SUBSYSTEM.getObjectLocation().isPresent() ? PRIMARY_CONTROLLER.getRightX() : 0,
    //       () -> {
    //         return VISION_SUBSYSTEM.getObjectLocation().orElse(null);
    //       },
    //       false,
    //       false).until(() -> VISION_SUBSYSTEM.shouldIntake()),
    return Commands.parallel(
      DRIVE_SUBSYSTEM.aimAtPointCommand(
        () -> -DRIVE_SUBSYSTEM.getPose().getRotation().plus(new Rotation2d(VISION_SUBSYSTEM.getObjectHeading().orElse(Units.Degrees.of(0)))).getCos() * (VISION_SUBSYSTEM.objectIsVisible() ? 0.75 : 0),
        () -> -DRIVE_SUBSYSTEM.getPose().getRotation().plus(new Rotation2d(VISION_SUBSYSTEM.getObjectHeading().orElse(Units.Degrees.of(0)))).getSin() * (VISION_SUBSYSTEM.objectIsVisible() ? 0.75 : 0),
        () -> 0,
        () -> VISION_SUBSYSTEM.getObjectLocation().orElse(null),
        false,
        false),

         //INTAKE_SUBSYSTEM.intakeCommand(),
         SHOOTER_SUBSYSTEM.intakeCommand()
     )
     .until(() -> SHOOTER_SUBSYSTEM.isObjectPresent());
   }

  /**
   * PARTY BUTTON!!!!
   * @return Command that spins the robot and moves the shooter up and down
   */
  private Command partyMode() {
    return Commands.parallel(
      DRIVE_SUBSYSTEM.driveCommand(() -> 0.0, () -> 0.0, () -> 1.0),
      SHOOTER_SUBSYSTEM.shootPartyMode()
    );
  }

  /**
   * Get correct speaker for current alliance
   * @return Location of appropriate speaker
   */
  private static AprilTag speakerSupplier() {
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
    m_automodeChooser.addOption("Just shoot", SHOOTER_SUBSYSTEM.shootSpeakerCommand().withTimeout(5));
    m_automodeChooser.addOption(Constants.AutoNames.CENTER_CLOSETOP_CLOSEMID_CLOSEBOTTOM_AUTO_NAME.getFirst(), new AutoTrajectory(DRIVE_SUBSYSTEM, Constants.AutoNames.CENTER_CLOSETOP_CLOSEMID_CLOSEBOTTOM_AUTO_NAME.getSecond()).getCommand());
    m_automodeChooser.addOption(Constants.AutoNames.CENTER_CLOSEBOTTOM_CLOSEMID_CLOSETOP_FARTOP_AUTO_NAME.getFirst(), new AutoTrajectory(DRIVE_SUBSYSTEM, Constants.AutoNames.CENTER_CLOSEBOTTOM_CLOSEMID_CLOSETOP_FARTOP_AUTO_NAME.getSecond()).getCommand());
    m_automodeChooser.addOption(Constants.AutoNames.RIGHT_FARBOTTOM_FARMIDBOTTOM_AUTO_NAME.getFirst(), new AutoTrajectory(DRIVE_SUBSYSTEM, Constants.AutoNames.RIGHT_FARBOTTOM_FARMIDBOTTOM_AUTO_NAME.getSecond()).getCommand());
    m_automodeChooser.addOption(Constants.AutoNames.LEFT_CLOSETOP_FARTOP_AUTO_NAME.getFirst(), new AutoTrajectory(DRIVE_SUBSYSTEM, Constants.AutoNames.LEFT_CLOSETOP_FARTOP_AUTO_NAME.getSecond()).getCommand());
    m_automodeChooser.addOption(Constants.AutoNames.LEFT_WAIT_FARTOP_AUTO_NAME.getFirst(), new AutoTrajectory(DRIVE_SUBSYSTEM, Constants.AutoNames.LEFT_WAIT_FARTOP_AUTO_NAME.getSecond()).getCommand());
    m_automodeChooser.addOption(Constants.AutoNames.RIGHT_FARDISRUPT_FARTOP_AUTO_NAME.getFirst(), new AutoTrajectory(DRIVE_SUBSYSTEM, Constants.AutoNames.RIGHT_FARDISRUPT_FARTOP_AUTO_NAME.getSecond()).getCommand());
      }

  /**
   * Run simlation related methods
   */
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  /**
   * Run checks in auto
   */
  public void autonomousPeriodic() {
    // stop auto if we go too far
    if (DRIVE_SUBSYSTEM.getAlliance().equals(Alliance.Blue)) {
      if (DRIVE_SUBSYSTEM.getPose().getX() > 9.73) Commands.run(() -> {}, DRIVE_SUBSYSTEM);
    }
    else if (DRIVE_SUBSYSTEM.getAlliance().equals(Alliance.Red)) {
      if (DRIVE_SUBSYSTEM.getPose().getX() < 6.84) Commands.run(() -> {}, DRIVE_SUBSYSTEM);
    }
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
   * Compose command to test intake, amp shoot, and speaker shoot
   * @return Command that does the intake command, amp shoot command, and speaker shoot command
   */
  public Command getTestCommand() {
    return Commands.sequence(
      Commands.print("Intaking..."),
      intakeCommand().withTimeout(Constants.AutoNames.TEST_COMMAND_TIME),
      Commands.print("Moving shooter down..."),
      SHOOTER_SUBSYSTEM.shootManualCommand(
        new State(ShooterSubsystem.ZERO_FLYWHEEL_SPEED, Units.Radians.of(Constants.Shooter.ANGLE_CONFIG.getLowerLimit()))
      ).withTimeout(Constants.AutoNames.TEST_COMMAND_TIME),
      Commands.print("Moving shooter up..."),
      SHOOTER_SUBSYSTEM.shootManualCommand(
        new State(ShooterSubsystem.ZERO_FLYWHEEL_SPEED, Units.Radians.of(Constants.Shooter.ANGLE_CONFIG.getUpperLimit()))
      ).withTimeout(Constants.AutoNames.TEST_COMMAND_TIME),
      Commands.print("Scoring amp..."),
      SHOOTER_SUBSYSTEM.scoreAmpCommand().withTimeout(Constants.AutoNames.TEST_COMMAND_TIME),
      Commands.print("Scoring speaker..."),
      SHOOTER_SUBSYSTEM.shootSpeakerCommand().withTimeout(Constants.AutoNames.TEST_COMMAND_TIME)
    );
  }

  /**
   * Configure default Shuffleboard tab
   */
  public void defaultShuffleboardTab() {
    Shuffleboard.selectTab(Constants.SmartDashboard.SMARTDASHBOARD_DEFAULT_TAB);
    autoModeChooser();
    SmartDashboard.putData(Constants.SmartDashboard.SMARTDASHBOARD_AUTO_MODE, m_automodeChooser);
    SmartDashboard.putNumber(Constants.SmartDashboard.SMARTDASHBOARD_SHOOTER_SPEED, 0.0);
    SmartDashboard.putNumber(Constants.SmartDashboard.SMARTDASHBOARD_SHOOTER_ANGLE, 0.0);
  }
}
