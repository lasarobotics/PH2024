// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** PurplePath Pose */
public class PurplePathPose {
  private final double MIN_FINAL_APPROACH_DISTANCE = 0.15;
  private final double MAX_FINAL_APPROACH_DISTANCE = 1.00;

  private Pose2d m_bluePose, m_redPose;
  private Pose2d m_blueFinalApproachPose, m_redFinalApproachPose;
  private PathPlannerPath m_blueFinalApproachPath, m_redFinalApproachPath;
  private double m_finalApproachDistance;

  /**
   * Create alliance specific goal for PurplePath
   * <p>
   * MUST call {@link PurplePathPose#calculateFinalApproach(PathConstraints)} before using
   * @param bluePose Pose if blue alliance
   * @param redPose Pose if red alliance
   * @param finalApproachDistance Distance of final approach
   * @param isReversed True if robot's rear is facing object
   */
  public PurplePathPose(Pose2d bluePose, Pose2d redPose, Measure<Distance> finalApproachDistance, boolean isReversed) {
    this.m_bluePose = bluePose;
    this.m_redPose = redPose;
    this.m_finalApproachDistance = MathUtil.clamp(
      finalApproachDistance.in(Units.Meters),
      MIN_FINAL_APPROACH_DISTANCE,
      MAX_FINAL_APPROACH_DISTANCE
    );

    Rotation2d finalApproachDirectionOffset = isReversed ? Rotation2d.fromRadians(0.0) : Rotation2d.fromRadians(Math.PI);

    this.m_blueFinalApproachPose = new Pose2d(
      bluePose.getTranslation()
        .plus(new Translation2d(m_finalApproachDistance, m_bluePose.getRotation().plus(finalApproachDirectionOffset))),
      bluePose.getRotation()
    );
    this.m_redFinalApproachPose = new Pose2d(
      redPose.getTranslation()
        .plus(new Translation2d(m_finalApproachDistance, m_redPose.getRotation().plus(finalApproachDirectionOffset))),
      redPose.getRotation()
    );
  }

  /**
   * Create alliance specific goal for PurplePath
   * <p>
   * MUST call {@link PurplePathPose#calculateFinalApproach(PathConstraints)} before using
   * @param bluePose Pose if blue alliance
   * @param redPose Pose if red alliance
   * @param finalApproachDistance Distance of final approach in meters [0.15, 1.00]
   */
  public PurplePathPose(Pose2d bluePose, Pose2d redPose, Measure<Distance> finalApproachDistance) {
    this(bluePose, redPose, finalApproachDistance, false);
  }

  /**
   * Create shared goal for PurplePath
   * <p>
   * MUST call {@link PurplePathPose#calculateFinalApproach(PathConstraints)} before using
   * @param pose Goal pose
   * @param finalApproachDistance Distance of final approach
   * @param isReversed True if robot's rear is facing object
   */
  public PurplePathPose(Pose2d pose, Measure<Distance> finalApproachDistance, boolean isReversed) {
    this(pose, pose, finalApproachDistance, isReversed);
  }

  /**
   * Create shared goal for PurplePath
   * <p>
   * MUST call {@link PurplePathPose#calculateFinalApproach(PathConstraints)} before using
   * @param pose Goal pose
   * @param finalApproachDistance Distance of final approach
   */
  public PurplePathPose(Pose2d pose, Measure<Distance> finalApproachDistance) {
    this(pose, finalApproachDistance, false);
  }

  /**
   * Get final approach distance
   * @return Final approach distance
   */
  public double getFinalApproachDistance() {
    return m_finalApproachDistance;
  }

  /**
   * Calculate final approach paths
   * @param pathConstraints Path constraints to apply to final approach paths
   */
  public void calculateFinalApproach(PathConstraints pathConstraints) {
    List<Translation2d> blueFinalApproachBezier = PathPlannerPath.bezierFromPoses(m_blueFinalApproachPose, m_bluePose);
    m_blueFinalApproachPath = new PathPlannerPath(
      blueFinalApproachBezier,
      pathConstraints,
      new GoalEndState(0.0, m_blueFinalApproachPose.getRotation())
    );

    List<Translation2d> redFinalApproachBezier = PathPlannerPath.bezierFromPoses(m_redFinalApproachPose, m_redPose);
    m_redFinalApproachPath = new PathPlannerPath(
      redFinalApproachBezier,
      pathConstraints,
      new GoalEndState(0.0, m_redFinalApproachPose.getRotation())
    );
  }

  /**
   * Get goal pose for current alliance
   * @return Goal pose
   */
  public Pose2d getGoalPose() {
    if (DriverStation.getAlliance().isEmpty()) return null;
    Alliance currentAlliance = DriverStation.getAlliance().get();
    switch (currentAlliance) {
      case Red:
        return m_redPose;
      case Blue:
      default:
        return m_bluePose;
    }
  }

  /**
   * Get final approach pose for current alliance
   * @return Final approach pose
   */
  public Pose2d getFinalApproachPose() {
    if (DriverStation.getAlliance().isEmpty()) return null;
    Alliance currentAlliance = DriverStation.getAlliance().get();
    switch (currentAlliance) {
      case Red:
        return m_redFinalApproachPose;
      case Blue:
      default:
        return m_blueFinalApproachPose;
    }
  }

  /**
   * Get final approach path for current alliance
   * @return Final approach path
   */
  public PathPlannerPath getFinalApproachPath() {
    if (DriverStation.getAlliance().isEmpty()) return null;
    Alliance currentAlliance = DriverStation.getAlliance().get();
    switch (currentAlliance) {
      case Red:
        return m_redFinalApproachPath;
      case Blue:
      default:
        return m_blueFinalApproachPath;
    }
  }
}
