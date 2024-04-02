// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.AprilTagCamera.AprilTagCameraResult;

public class VisionSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    AprilTagCamera[] cameras;
    ObjectCamera objectCamera;

    public Hardware(ObjectCamera objectCamera, AprilTagCamera... cameras) {
      this.cameras = cameras;
      this.objectCamera = objectCamera;
    }

    public Hardware(AprilTagCamera... cameras) {
      this.cameras = cameras;
    }
  }

  private static VisionSubsystem m_subsystem;

  private static final String VISIBLE_TAGS_LOG_ENTRY = "/VisibleTags";
  private static final String ESTIMATED_POSES_LOG_ENTRY = "/EstimatedPoses";
  private static final String OBJECT_DISTANCE_LOG_ENTRY = "/ObjectDistance";
  private static final String OBJECT_HEADING_LOG_ENTRY = "/ObjectHeading";
  private static final String OBJECT_POSE_LOG_ENTRY = "/ObjectPose";
  private static final String OBJECT_DETECTED_LOG_ENTRY = "/ObjectDetected";

  private static final double INTAKE_YAW_TOLERANCE = 1;

  private AtomicReference<List<AprilTagCameraResult>> m_estimatedRobotPoses;
  private AtomicReference<List<AprilTag>> m_visibleTags;
  private AtomicReference<List<Pose2d>> m_loggedEstimatedPoses;
  private AtomicReference<List<Pose3d>> m_visibleTagPoses;

  private ObjectCamera m_objectCamera;
  private AprilTagCamera[] m_apriltagCameras;
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
    this.m_apriltagCameras = visionHardware.cameras;
    this.m_objectCamera = visionHardware.objectCamera;
    this.m_estimatedRobotPoses = new AtomicReference<List<AprilTagCameraResult>>();
    this.m_visibleTags = new AtomicReference<List<AprilTag>>();
    this.m_loggedEstimatedPoses = new AtomicReference<List<Pose2d>>();
    this.m_visibleTagPoses = new AtomicReference<List<Pose3d>>();

    this.m_sim = new VisionSystemSim(getName());

    // Load AprilTag field layout
    m_fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // PV estimates will always be blue
    m_fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

    // Set field layout for sim
    m_sim.addAprilTags(m_fieldLayout);

    // Setup camera pose estimation threads
    this.m_cameraNotifier = (RobotBase.isReal())
    ? new Notifier(() -> {
      for (var camera : m_apriltagCameras) camera.run();
      updateEstimatedGlobalPoses();
    })
    : new Notifier(() -> {
      if (m_poseSupplier != null) m_sim.update(m_poseSupplier.get());
      for (var camera : m_apriltagCameras) camera.run();
      updateEstimatedGlobalPoses();
    });

    // Set all cameras to primary pipeline
    for (var camera : m_apriltagCameras) camera.setPipelineIndex(0);

    // Add object camera to sim
    m_sim.addCamera(m_objectCamera.getCameraSim(), m_objectCamera.getTransform());

    // Add game object target to sim
    m_sim.addVisionTargets(m_objectCamera.getTargetSim());

    // Add AprilTag cameras to sim
    for (var camera : m_apriltagCameras) m_sim.addCamera(camera.getCameraSim(), camera.getTransform());

    // Start camera thread
    m_cameraNotifier.setName(getName());
    m_cameraNotifier.startPeriodic(GlobalConstants.ROBOT_LOOP_PERIOD);
  }

  /**
   * Initialize hardware devices for vision subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  private static Hardware initializeHardware() {
    Hardware visionHardware = new Hardware(
      new ObjectCamera(
        Constants.VisionHardware.CAMERA_OBJECT_NAME,
        Constants.VisionHardware.CAMERA_OBJECT_LOCATION,
        Constants.VisionHardware.CAMERA_OBJECT_RESOLUTION,
        Constants.VisionHardware.CAMERA_OBJECT_FOV
      ),
      new AprilTagCamera(
        Constants.VisionHardware.CAMERA_A_NAME,
        Constants.VisionHardware.CAMERA_A_LOCATION,
        Constants.VisionHardware.CAMERA_A_RESOLUTION,
        Constants.VisionHardware.CAMERA_A_FOV
      ),
      new AprilTagCamera(
        Constants.VisionHardware.CAMERA_B_NAME,
        Constants.VisionHardware.CAMERA_B_LOCATION,
        Constants.VisionHardware.CAMERA_B_RESOLUTION,
        Constants.VisionHardware.CAMERA_B_FOV
      )
    );

    return visionHardware;
  }

   /**
   * Update currently estimated robot pose from each camera
   */
  private void updateEstimatedGlobalPoses() {
    var apriltagCameraResult = new ArrayList<AprilTagCameraResult>();

    var visibleTags = new ArrayList<AprilTag>();
    var loggedPoses = new ArrayList<Pose2d>();
    var visibleTagPoseList = new ArrayList<Pose3d>();
    for (var camera : m_apriltagCameras) {
      var result = camera.getLatestEstimatedPose();
      if (result == null) continue;
      result.estimatedRobotPose.targetsUsed.forEach((photonTrackedTarget) -> {
        var tag = getTag(photonTrackedTarget.getFiducialId());
        if (tag.isPresent()) {
          visibleTags.add(tag.get());
          visibleTagPoseList.add(tag.get().pose);
        }
      });
      apriltagCameraResult.add(result);
      loggedPoses.add(result.estimatedRobotPose.estimatedPose.toPose2d());
    }

    // Log visible tags and estimated poses
    m_visibleTagPoses.set(visibleTagPoseList);
    m_loggedEstimatedPoses.set(loggedPoses);

    m_visibleTags.set(visibleTags);
    m_estimatedRobotPoses.set(apriltagCameraResult);
  }

  /**
   * Get instance of vision subsystem, creating if nonexistent
   * @return Instance of vision subsystem
   */
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
    var objectLocation = getObjectLocation();
    Logger.recordOutput(getName() + OBJECT_DETECTED_LOG_ENTRY, getObjectLocation().isPresent());
    if (objectLocation.isEmpty()) return;
    Logger.recordOutput(getName() + "/shouldIntake", shouldIntake());
    Logger.recordOutput(getName() + OBJECT_POSE_LOG_ENTRY, objectLocation.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run in simulation
    var objectLocation = getObjectLocation();
    if (objectLocation.isEmpty()) return;

    Logger.recordOutput(getName() + OBJECT_POSE_LOG_ENTRY, objectLocation.get());
  }

  /**
   * Get currently estimated robot poses from each camera
   * @return List of estimated poses, the timestamp, and targets used to create the estimate
   */
  public List<AprilTagCameraResult> getEstimatedGlobalPoses() {
    Logger.recordOutput(getName() + VISIBLE_TAGS_LOG_ENTRY, m_visibleTagPoses.get().toArray(new Pose3d[0]));
    Logger.recordOutput(getName() + ESTIMATED_POSES_LOG_ENTRY, m_loggedEstimatedPoses.get().toArray(new Pose2d[0]));

    return m_estimatedRobotPoses.getAndSet(Collections.emptyList());
  }

  /**
   * Get IDs of currently visible tags
   * @return List of IDs of currently visible tags
   */
  public List<AprilTag> getVisibleTags() {
    return m_visibleTags.get();
  }

  public Optional<AprilTag> getTag(int id) {
    return m_fieldLayout.getTags().stream().filter((tag) -> tag.ID == id).findFirst();
  }

  /**
   * Get the position of an object that can be seen by the object camera.
   * @return The position of the object, relative to the field
   */
  public Optional<Translation2d> getObjectLocation() {

    Optional<Measure<Angle>> yaw = m_objectCamera.getYaw();
    Optional<Measure<Distance>> distance = m_objectCamera.getDistance();
    Pose2d pose = m_poseSupplier.get();
    if (yaw.isEmpty() || distance.isEmpty() || pose == null) return Optional.empty();

    Logger.recordOutput(getName() + OBJECT_DISTANCE_LOG_ENTRY, distance.get());
    Logger.recordOutput(getName() + OBJECT_HEADING_LOG_ENTRY, yaw.get());
    return Optional.of(pose.getTranslation().plus(
      new Translation2d(
        // distance.get().in(Units.Meters),
        1,
        Rotation2d.fromRadians(pose.getRotation().getRadians() - yaw.get().in(Units.Radians))
      )
    ));
  }

  /**
   * Gets the object heading, relative to the camera.
   * @return the heading
   */
  public Optional<Measure<Angle>> getObjectHeading() {
    Optional<Measure<Angle>> yaw = m_objectCamera.getYaw();
    if (yaw.isEmpty()) return Optional.empty();
    return yaw;
  }

  public boolean shouldIntake() {
    if (!m_objectCamera.objectIsVisible()) return false;
    double angle = m_objectCamera.getYaw().orElse(Units.Degrees.of(INTAKE_YAW_TOLERANCE)).in(Units.Degrees);
    Logger.recordOutput(getName() + "/angle111", angle);
    return Math.abs(angle) < INTAKE_YAW_TOLERANCE;
  }

  public boolean objectIsVisible() {
    return m_objectCamera.objectIsVisible();
  }

  @Override
  public void close() {
    for (var camera : m_apriltagCameras) camera.close();
    m_objectCamera.close();
    m_cameraNotifier.close();
  }
}
