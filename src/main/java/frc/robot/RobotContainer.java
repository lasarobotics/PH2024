// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
  private static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(
    DriveSubsystem.initializeHardware(),
    Constants.Drive.DRIVE_TURN_PID,
    Constants.Drive.DRIVE_CONTROL_CENTRICITY,
    Constants.Drive.DRIVE_TURN_SCALAR,
    Constants.HID.CONTROLLER_DEADBAND,
    Constants.Drive.DRIVE_LOOKAHEAD,
    Constants.Drive.DRIVE_SLIP_RATIO,
    Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_INPUT_CURVE
  );

  private static final CommandXboxController PRIMARY_CONTROLLER = new CommandXboxController(Constants.HID.PRIMARY_CONTROLLER_PORT);

  public RobotContainer() {
    // Set drive command
    DRIVE_SUBSYSTEM.setDefaultCommand(
      DRIVE_SUBSYSTEM.run(
        () -> DRIVE_SUBSYSTEM.teleopPID(-PRIMARY_CONTROLLER.getLeftY(), -PRIMARY_CONTROLLER.getLeftX(), PRIMARY_CONTROLLER.getRightX())
      )
    );

    // Setup AutoBuilder
    DRIVE_SUBSYSTEM.configureAutoBuilder();

    // Bind buttons and triggers
    configureBindings();
  }

  private void configureBindings() {
    PRIMARY_CONTROLLER.start().onTrue(DRIVE_SUBSYSTEM.runOnce(() -> DRIVE_SUBSYSTEM.toggleTractionControl()));
    PRIMARY_CONTROLLER.leftBumper().whileTrue(
      DRIVE_SUBSYSTEM.run(() ->
        DRIVE_SUBSYSTEM.aimAtPoint(
          -PRIMARY_CONTROLLER.getLeftY(),
          -PRIMARY_CONTROLLER.getLeftX(),
          Constants.Field.CENTER
        )
      ).finallyDo(() -> DRIVE_SUBSYSTEM.resetTurnPID())
    );

    PRIMARY_CONTROLLER.rightBumper().whileTrue(DRIVE_SUBSYSTEM.goToPose(Constants.Field.SUBSTATION));
    PRIMARY_CONTROLLER.a().whileTrue(DRIVE_SUBSYSTEM.goToPose(Constants.Field.GRID_5));
    PRIMARY_CONTROLLER.b().whileTrue(DRIVE_SUBSYSTEM.goToPose(Constants.Field.GRID_8));
    PRIMARY_CONTROLLER.x().whileTrue(DRIVE_SUBSYSTEM.goToPose(Constants.Field.GRID_2));
  }

  /**
   * Run simlation related methods
   */
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  /**
   * Get currently selected autonomous command
   * @return Autonomous command
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
