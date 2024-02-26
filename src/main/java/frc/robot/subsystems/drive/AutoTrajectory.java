// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoTrajectory {
  DriveSubsystem m_driveSubsystem;
  Command m_swerveCommand;
  Pair<String,List<PathPlannerPath>> m_auto;

  /**
   * Create new path trajectory using PathPlanner path
   * @param driveSubsystem DriveSubsystem to drive the robot
   * @param autoName PathPlanner auto name
   */
  public AutoTrajectory(DriveSubsystem driveSubsystem, String autoName) {
    this.m_driveSubsystem = driveSubsystem;

    // Get path
    m_auto = new Pair<String, List<PathPlannerPath>>(autoName, PathPlannerAuto.getPathGroupFromAutoFile(autoName));
  }

  /**
   * Creates new path trajectory using a physical x,y coordinate points
   * @param driveSubsystem DriveSubsystem required for drivetrain movement
   * @param waypoints List of x, y coordinate pairs in trajectory
   * @param pathConstraints Path following constraints
   */
  public AutoTrajectory(DriveSubsystem driveSubsystem, List<Pose2d> waypoints, PathConstraints pathConstraints) {
    this.m_driveSubsystem = driveSubsystem;

    // Generate path from waypoints
    m_auto = new Pair<String, List<PathPlannerPath>>("", List.of(new PathPlannerPath(
      PathPlannerPath.bezierFromPoses(waypoints),
      pathConstraints,
      new GoalEndState(0.0, waypoints.get(waypoints.size() - 1).getRotation())
    )));
  }

  /** Return initial pose */
  public Pose2d getInitialPose() {
    return m_auto.getSecond().get(0).getPreviewStartingHolonomicPose();
  }

  /**
   * Get auto command to execute path
   * @return Auto command group that will stop when complete
   */
  public Command getCommand() {
    Command autoCommand = m_auto.getSecond().size() == 1
      ? AutoBuilder.followPath(m_auto.getSecond().get(0))
      : new PathPlannerAuto(m_auto.getFirst());

    return Commands.sequence(
      m_driveSubsystem.resetPoseCommand(() -> getInitialPose()),
      autoCommand,
      m_driveSubsystem.stopCommand(),
      m_driveSubsystem.lockCommand()
    );
  }
}