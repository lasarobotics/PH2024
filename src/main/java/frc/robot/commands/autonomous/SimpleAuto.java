// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.AutoTrajectory;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SimpleAuto extends SequentialCommandGroup {
  /** Creates a new Leave. */
  public SimpleAuto(DriveSubsystem driveSubsystem) {
    List<Pose2d> path = List.of(
      new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)),
      new Pose2d(new Translation2d(3.2, 0.0), Rotation2d.fromDegrees(0.0))
    );
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    AutoTrajectory auto = new AutoTrajectory(driveSubsystem, path, constraints);

    addCommands(
      auto.getCommand()
    );
  }
}
