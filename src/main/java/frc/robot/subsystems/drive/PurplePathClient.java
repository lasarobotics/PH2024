// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.JSONObject;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private HttpURLConnection m_serverConnection;
  private boolean m_isConnected;
  private boolean m_connectivityCheckEnabled;
  private Notifier m_periodicNotifier;

  public PurplePathClient(DriveSubsystem driveSubsystem) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_isConnected = false;
    this.m_connectivityCheckEnabled = true;

    // Set URI
    if (RobotBase.isSimulation()) URI = "http://localhost:5000/";
    else URI = "http://purplebox.local:5000/";

    // Initialize connectivity check thread
    this.m_periodicNotifier = new Notifier(() -> periodic());

    // Start connectivity check thread
    m_periodicNotifier.setName(getClass().getSimpleName());
    m_periodicNotifier.startPeriodic(GlobalConstants.ROBOT_LOOP_PERIOD);
  }

  /**
   * Send JSON request to PurplePath server
   * @param jsonRequest JSON array string of start and goal point
   * @return PurplePath server response
   * @throws IOException
   */
  private String sendRequest(String jsonRequest) throws IOException {
    String jsonResponse = "";

    // Define the server endpoint to send the HTTP request to
    m_serverConnection = (HttpURLConnection)new URL(URI).openConnection();

    // Indicate that we want to write to the HTTP request body
    m_serverConnection.setDoOutput(true);
    m_serverConnection.setRequestMethod("POST");
    m_serverConnection.setRequestProperty("Content-Type", "application/json");

    // Writing the post data to the HTTP request body
    BufferedWriter httpRequestBodyWriter = new BufferedWriter(new OutputStreamWriter(m_serverConnection.getOutputStream()));
    httpRequestBodyWriter.write(jsonRequest);
    httpRequestBodyWriter.close();

    // Reading from the HTTP response body
    Scanner httpResponseScanner = new Scanner(m_serverConnection.getInputStream());
    while (httpResponseScanner.hasNextLine()) jsonResponse += httpResponseScanner.nextLine();
    m_isConnected = m_serverConnection.getResponseCode() == 200;
    httpResponseScanner.close();

    // Return response
    return jsonResponse;
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

    // Construct JSON request
    String jsonRequest = isClose ? JSONObject.writePointList(Arrays.asList(start.getTranslation(), goalPose.getTranslation()))
                                 : JSONObject.writePointList(Arrays.asList(start.getTranslation(), finalApproachPose.getTranslation()));

    // Send pathfinding request and get response
    String jsonResponse = "";
    try {
      jsonResponse = sendRequest(jsonRequest);
    } catch (IOException e) {
      System.out.println(e.getMessage());
      m_isConnected = false;
      return Commands.none();
    }

    // Attempt to read path from response
    List<Translation2d> points = JSONObject.readPointList(jsonResponse);

    // If path isn't there, return specified command
    if (points == null || points.size() < 2) return parallelCommand;

    // Convert to PathPoint list
    double distance = 0.0;
    List<PathPoint> waypoints = new ArrayList<>();
    for (int i = 0; i < points.size(); i++) {
      waypoints.add(new PathPoint(points.get(i), new RotationTarget(0.0, goalPose.getRotation())));
      distance += points.get(i).getDistance(points.get(MathUtil.clamp(i - 1, 0, points.size())));
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
  private void periodic() {
    if (!m_connectivityCheckEnabled) return;
    if (m_isConnected) return;

    try {
      sendRequest(JSONObject.writePointList(Arrays.asList(new Translation2d(), new Translation2d())));
    } catch (IOException e) {
      m_isConnected = false;
    }
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
