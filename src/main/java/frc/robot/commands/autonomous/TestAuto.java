// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.AutoTrajectory;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TestAuto extends SequentialCommandGroup {
  /** Creates a new Leave auto */
  public TestAuto(DriveSubsystem driveSubsystem) {
    AutoTrajectory auto = new AutoTrajectory(driveSubsystem, "Test");

    addCommands(
      auto.getCommand()
    );
  }
}
