// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoTrajectory {
  DriveSubsystem m_driveSubsystem;
  Command m_swerveCommand;
  PathPlannerPath m_path;

  /**
   * Create new path trajectory using PathPlanner path
   * @param driveSubsystem DriveSubsystem to drive the robot
   * @param pathName PathPlanner path name
   */
  public AutoTrajectory(DriveSubsystem driveSubsystem, String pathName) {
    this.m_driveSubsystem = driveSubsystem;

    // Get path
    m_path = PathPlannerPath.fromPathFile(pathName);
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
    m_path = new PathPlannerPath(
      PathPlannerPath.bezierFromPoses(waypoints),
      pathConstraints,
      new GoalEndState(0.0, waypoints.get(waypoints.size() - 1).getRotation())
    );
  }

  /**
   * Get markers of path
   * @return A list of markers within the path
   */
  public List<EventMarker> getEventMarkers() {
    return m_path.getEventMarkers();
  }

  /**
   * Get initial pose for path
   * @return Path initial pose
   */
  public Pose2d getInitalPose() {
    if (m_path == null) return new Pose2d();
    return new Pose2d(m_path.getPoint(0).position, m_path.getPoint(0).rotationTarget.getTarget());
  }

  /**
   * Get Ramsete command to run
   * @return Ramsete command that will stop when complete
   */
  public Command getCommandAndStop() {
    return AutoBuilder.followPath(m_path)
            .andThen(() -> m_driveSubsystem.resetRotatePID())
            .andThen(m_driveSubsystem.stopCommand())
            .andThen(m_driveSubsystem.lockCommand());
  }

  /**
   * Get auto command to execute path
   * @return Ramsete command that does NOT stop when complete
   */
  public Command getCommand() {
    return AutoBuilder.followPath(m_path).andThen(() -> m_driveSubsystem.resetRotatePID());
  }
}