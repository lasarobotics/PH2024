// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.AutoTrajectory;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Simple extends SequentialCommandGroup {
  /** Creates a new Leave. */
  public Simple(DriveSubsystem driveSubsystem) {
    String pathName = "Simple-Blue";
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            pathName = "Simple-Red";
        }
    }

    addCommands(
      new AutoTrajectory(driveSubsystem, pathName).getCommandAndStop()
    );
  }
}
