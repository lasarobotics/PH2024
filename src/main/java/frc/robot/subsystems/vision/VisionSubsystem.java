// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.function.Supplier;

import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    VisionCamera[] cameras;

    public Hardware(VisionCamera... cameras) {
      this.cameras = cameras;
    }
  }

  private static VisionSubsystem m_subsystem;

  private static final String VISIBLE_TAGS_LOG_ENTRY = "/VisibleTags";
  private static final String ESTIMATED_POSES_LOG_ENTRY = "/EstimatedPoses";

  private VisionCamera[] m_cameras;
  private Notifier m_cameraNotifier;
  private AprilTagFieldLayout m_fieldLayout;

  private Supplier<Pose2d> m_poseSupplier;

  private VisionSystemSim m_sim;

  /**
   * Create a new vision subsystem
   * @param visionHardware Vision hardware
   */
  private VisionSubsystem(Hardware visionHardware) {
    setName(getClass().getSimpleName());
    this.m_cameras = visionHardware.cameras;
    this.m_sim = new VisionSystemSim(getName());

    // Load AprilTag field layout
    m_fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    // PV estimates will always be blue
    m_fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

    // Set field layout for sim
    m_sim.addAprilTags(m_fieldLayout);

    // Setup camera pose estimation threads
    this.m_cameraNotifier = (RobotBase.isReal()) ? new Notifier(() -> { for (var camera : m_cameras) camera.run(); })
                                                 : new Notifier(() -> {
                                                     if (m_poseSupplier != null) m_sim.update(m_poseSupplier.get());
                                                     for (var camera : m_cameras) camera.run();
                                                   });

    // Set all cameras to primary pipeline
    for (var camera : m_cameras) camera.setPipelineIndex(0);

    // Add cameras to sim
    for (var camera : m_cameras) m_sim.addCamera(camera.getCameraSim(), camera.getTransform());

    // Start camera thread
    m_cameraNotifier.setName(getName());
    m_cameraNotifier.startPeriodic(GlobalConstants.ROBOT_LOOP_PERIOD);
  }

  public static Hardware initializeHardware() {
    Hardware visionHardware = new Hardware(
      new VisionCamera(
        Constants.VisionHardware.CAMERA_A_NAME,
        Constants.VisionHardware.CAMERA_A_LOCATION,
        Constants.VisionHardware.CAMERA_A_RESOLUTION,
        Constants.VisionHardware.CAMERA_A_FOV
      ),
      new VisionCamera(
        Constants.VisionHardware.CAMERA_B_NAME,
        Constants.VisionHardware.CAMERA_B_LOCATION,
        Constants.VisionHardware.CAMERA_B_RESOLUTION,
        Constants.VisionHardware.CAMERA_B_FOV
      ),
      new VisionCamera(
        Constants.VisionHardware.CAMERA_C_NAME,
        Constants.VisionHardware.CAMERA_C_LOCATION,
        Constants.VisionHardware.CAMERA_C_RESOLUTION,
        Constants.VisionHardware.CAMERA_C_FOV
      )
    );

    return visionHardware;
  }

  public static VisionSubsystem getInstance() {
    if (m_subsystem == null) m_subsystem = new VisionSubsystem(initializeHardware());
    return m_subsystem;
  }

  /**
   * Set pose supplier for simulation
   * @param poseSupplier Pose supplier from drive subsystem
   */
  public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
    m_poseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run in simulation
  }

  /**
   * Get currently estimated robot poses from each camera
   * @return List of estimated poses, the timestamp, and targets used to create the estimate
   */
  public List<EstimatedRobotPose> getEstimatedGlobalPoses() {
    List<EstimatedRobotPose> estimatedPoses = new ArrayList<EstimatedRobotPose>();

    HashSet<Pose3d> visibleTags = new HashSet<Pose3d>();
    List<Pose2d> loggedPoses = new ArrayList<Pose2d>();
    for (var camera : m_cameras) {
      var result = camera.getLatestEstimatedPose();
      if (result == null) continue;
      result.targetsUsed.forEach((photonTrackedTarget) -> {
        visibleTags.add(m_fieldLayout.getTagPose(photonTrackedTarget.getFiducialId()).get());
      });
      estimatedPoses.add(result);
      loggedPoses.add(result.estimatedPose.toPose2d());
    }

    // Log visible tags and estimated poses
    Logger.recordOutput(getName() + VISIBLE_TAGS_LOG_ENTRY, visibleTags.toArray(new Pose3d[0]));
    Logger.recordOutput(getName() + ESTIMATED_POSES_LOG_ENTRY, loggedPoses.toArray(new Pose2d[0]));

    return estimatedPoses;
  }

  @Override
  public void close() {
    for (var camera : m_cameras) camera.close();
    m_cameraNotifier.close();
  }
}
