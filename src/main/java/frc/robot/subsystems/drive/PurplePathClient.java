// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import org.lasarobotics.hardware.PurpleManager;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/** PurplePath Client */
public class PurplePathClient {
  private final String GOAL_POSE_LOG_ENTRY = "/GoalPose";
  private final String FINAL_APPROACH_POSE_LOG_ENTRY = "/FinalApproachPose";
  private final double FINAL_APPROACH_SPEED_FUDGE_FACTOR = 0.6;

  private final String URI;

  private DriveSubsystem m_driveSubsystem;
  private boolean m_isConnected;
  private boolean m_connectivityCheckEnabled;
  private Notifier m_periodicNotifier;
  private NetworkTableInstance m_purplePath;
  private StructArraySubscriber<Translation2d> m_pathSub;
  private StructPublisher<Translation2d> m_startPub;
  private StructPublisher<Translation2d> m_endPub;

  private NetworkTable m_NetworkTable;


  public PurplePathClient(DriveSubsystem driveSubsystem) {
    this.m_isConnected = false;
    this.m_driveSubsystem = driveSubsystem;
    this.m_purplePath = NetworkTableInstance.getDefault();
    this.m_purplePath.startServer();
    // Set URI
    if (RobotBase.isSimulation()) URI = "http://localhost:5000/";
    else URI = "http://purplebox.local:5000/";

    // Initialize connectivity check thread
    this.m_periodicNotifier = new Notifier(() -> periodic());

    Translation2d[] temp = {new Translation2d(0.0, 0.0), new Translation2d(0.0, 0.0)};

    m_NetworkTable = m_purplePath.getTable("SmartDashboard");

    m_pathSub = m_NetworkTable.getStructArrayTopic("Path", Translation2d.struct).subscribe(temp);
    m_startPub = m_NetworkTable.getStructTopic("Start Point", Translation2d.struct).publish();
    m_endPub = m_NetworkTable.getStructTopic("End Point", Translation2d.struct).publish();

    m_startPub.setDefault(new Translation2d(0.0, 0.0));
    m_endPub.setDefault(new Translation2d(0.0, 0.0));

    PurpleManager.addCallback(this::periodic);

  }

  /**
   * Get PathPlanner command to drive robot
   * @param path Desired robot path
   * @return Underlying PathPlanner command to use
   */
  private Command getPathPlannerCommand(PathPlannerPath path) {
    return new FollowPathHolonomic(
      path,
      m_driveSubsystem::getPose,
      m_driveSubsystem::getChassisSpeeds,
      m_driveSubsystem::autoDrive,
      m_driveSubsystem.getPathFollowerConfig(),
      () -> false,
      m_driveSubsystem
    );
  }

  /**
   * Get trajectory command to go from start pose to goal pose
   * @param start Starting pose of robot
   * @param goal Desired pose of robot
   * @return Command that drives robot to desired pose
   */
  private Command getCommand(Pose2d start, PurplePathPose goal, Command parallelCommand) {
    CommandScheduler.getInstance().removeComposedCommand(parallelCommand);

    Pose2d goalPose = goal.getGoalPose();
    Pose2d finalApproachPose = goal.getFinalApproachPose();
    PathPlannerPath finalApproachPath = goal.getFinalApproachPath();
    double finalApproachDistance = goal.getFinalApproachDistance();

    if (goalPose == null || finalApproachPose == null || finalApproachPath == null) return parallelCommand;

    // Check if robot is close to goal
    boolean isClose = start.getTranslation().getDistance(goalPose.getTranslation()) < finalApproachDistance;

    m_endPub.set(goalPose.getTranslation());
    Translation2d points[] = m_pathSub.get();
    if(points.length != 2){
    while(points[points.length-1].getX() != goalPose.getTranslation().getX() && points[points.length-1].getY() != goalPose.getTranslation().getY()){
      points = m_pathSub.get();
    } 
  } else {
    return parallelCommand;
  }
    System.out.println(points);

    // Convert to PathPoint list
    double distance = 0.0;
    List<PathPoint> waypoints = new ArrayList<>();
    for (int i = 0; i < points.length; i++) {
      waypoints.add(new PathPoint(points[i], new RotationTarget(0.0, goalPose.getRotation())));
      distance += points[i].getDistance(points[(MathUtil.clamp(i - 1, 0, points.length))]);
    }

    // Generate path
    PathPlannerPath path = PathPlannerPath.fromPathPoints(
      waypoints,
      m_driveSubsystem.getPathConstraints(),
      new GoalEndState(
        isClose ? 0.0
                : Math.min(
                    Math.sqrt(2 * m_driveSubsystem.getPathConstraints().getMaxAccelerationMpsSq() * finalApproachDistance) * FINAL_APPROACH_SPEED_FUDGE_FACTOR,
                    Math.sqrt(2 * m_driveSubsystem.getPathConstraints().getMaxAccelerationMpsSq() * distance)
                ),
        finalApproachPose.getRotation()
      )
    );

    Logger.recordOutput(getClass().getSimpleName() + GOAL_POSE_LOG_ENTRY, goalPose);
    Logger.recordOutput(getClass().getSimpleName() + FINAL_APPROACH_POSE_LOG_ENTRY, finalApproachPose);

    // Return path following command

    return isClose ? getPathPlannerCommand(path).alongWith(parallelCommand)
                    : Commands.sequence(
                      getPathPlannerCommand(path),
                      getPathPlannerCommand(finalApproachPath).alongWith(parallelCommand)
                     );
  }

  /**
   * Call this method periodically
   */
  public void periodic() {
    Translation2d start_point = m_driveSubsystem.getPose().getTranslation();
    m_startPub.set(start_point);
    m_isConnected = m_purplePath.isConnected();

  }

  /**
   * Get command to execute trajectory
   * @param goal Goal pose
   * @param parallelCommand Command to run in parallel on final approach
   * @return Trajectory command
   */
  public Command getTrajectoryCommand(PurplePathPose goal, Command parallelCommand) {
    return getCommand(m_driveSubsystem.getPose(), goal, parallelCommand);
  }

  /**
   * Get command to execute trajectory
   * @param goal Goal pose
   * @return Trajectory command
   */
  public Command getTrajectoryCommand(PurplePathPose goal) {
    return getTrajectoryCommand(goal, Commands.none());
  }

  /**
   * Get if connected to coprocessor
   * @return True if connected to PurplePath server on coprocessor
   */
  public boolean isConnected() {
    return m_isConnected;
  }

  /**
   * Enable connectivity check
   */
  public void enableConnectivityCheck() {
    m_connectivityCheckEnabled = true;
  }

  /**
   * Disable connectivity check
   */
  public void disableConnectivityCheck() {
    m_connectivityCheckEnabled = false;
  }
}
