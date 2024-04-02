// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Create a camera */
public class AprilTagCamera implements Runnable, AutoCloseable {
  private final double APRILTAG_POSE_AMBIGUITY_THRESHOLD = 0.2;
  private final double POSE_MAX_HEIGHT = 0.75;
  private final Measure<Distance> MAX_TAG_DISTANCE = Units.Meters.of(100.0);

  public static class AprilTagCameraResult {
    public final EstimatedRobotPose estimatedRobotPose;
    public final Matrix<N3, N1> visionMeasurementStdDevs;

    public AprilTagCameraResult(EstimatedRobotPose estimatedRobotPose,
                                Matrix<N3, N1> visionMeasurementStdDevs) {
      this.estimatedRobotPose = estimatedRobotPose;
      this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    }
  }

  public enum Resolution {
    RES_320_240(320, 240),
    RES_640_480(640, 480),
    RES_1280_720(1280, 720),
    RES_1280_800(1280, 800),
    RES_1920_1080(1920, 1080);

    public final int width;
    public final int height;

    private Resolution(int width, int height) {
      this.width = width;
      this.height = height;
    }
  }

  private PhotonCamera m_camera;
  private PhotonCameraSim m_cameraSim;
  private PhotonPoseEstimator m_poseEstimator;
  private Transform3d m_transform;
  private AtomicReference<AprilTagCameraResult> m_atomicEstimatedRobotPose;

  /**
   * Create VisionCamera
   * @param name Name of device
   * @param transform Location on robot in meters
   * @param resolution Resolution used by camera
   * @param fovDiag Diagonal FOV of camera
   */
  public AprilTagCamera(String name, Transform3d transform, Resolution resolution, Rotation2d fovDiag) {
    this.m_camera = new PhotonCamera(name);
    this.m_transform = transform;
    var fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // PV estimates will always be blue
    fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.m_poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, m_transform);
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    this.m_atomicEstimatedRobotPose = new AtomicReference<AprilTagCameraResult>();

    // Create simulated AprilTag camera
    var cameraProperties = SimCameraProperties.PERFECT_90DEG();
    cameraProperties.setCalibration(resolution.width, resolution.height, fovDiag);
    this.m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties);

    // Enable wireframe in sim camera stream
    m_cameraSim.enableDrawWireframe(true);
  }

  /**
   * Get camera sim
   * @return Simulated camera object
   */
  PhotonCameraSim getCameraSim() {
    return m_cameraSim;
  }

  /**
   * Check if pose is valid
   * @param pose Pose to check
   * @return True if pose is valid
   */
  private boolean isEstimatedPoseValid(Pose3d pose) {
    // Make sure pose is on the field
    if (pose.getX() < 0.0 || pose.getX() >= Constants.Field.FIELD_LENGTH
     || pose.getY() < 0.0 || pose.getY() >= Constants.Field.FIELD_WIDTH) return false;

    // Make sure pose is near the floor
    if (pose.getZ() > POSE_MAX_HEIGHT) return false;

    // Pose is acceptable
    return true;
  }

  @Override
  public void run() {
    // Return if camera or field layout failed to load
    if (m_poseEstimator == null || m_camera == null) return;

    // Put camera connected indicator on SmartDashboard
    SmartDashboard.putBoolean(m_camera.getName(), m_camera.isConnected());

    // Return if camera is not connected
    if (!m_camera.isConnected()) return;

    // Update and log inputs
    PhotonPipelineResult pipelineResult = m_camera.getLatestResult();

    // Return if result is non-existent or invalid
    if (!pipelineResult.hasTargets()) return;
    if (pipelineResult.targets.size() == 1
        && pipelineResult.targets.get(0).getPoseAmbiguity() > APRILTAG_POSE_AMBIGUITY_THRESHOLD) return;

    // Update pose estimate
    m_poseEstimator.update(pipelineResult).ifPresent(estimatedRobotPose -> {
        // Make sure the measurement is valid
        if (!isEstimatedPoseValid(estimatedRobotPose.estimatedPose)) return;

        // Get distance to closest tag
        var closestTagDistance = MAX_TAG_DISTANCE;
        for (var target : estimatedRobotPose.targetsUsed) {
          var tagDistance = Units.Meters.of(target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d()));
          if (tagDistance.lte(closestTagDistance)) closestTagDistance = tagDistance;
        }

        // Calculate standard deviation
        double xyStdDev = 0.01 * Math.pow(closestTagDistance.in(Units.Meters), 2.0) / estimatedRobotPose.targetsUsed.size();
        double thetaStdDev = 0.01 * Math.pow(closestTagDistance.in(Units.Meters), 2.0) / estimatedRobotPose.targetsUsed.size();

        // Set result
        var result = new AprilTagCameraResult(
          estimatedRobotPose,
          VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
        );
        m_atomicEstimatedRobotPose.set(result);
    });
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once.
   * If it returns a non-null value, it is a new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance.
   * @return Latest estimated pose
   */
  public AprilTagCameraResult getLatestEstimatedPose() {
    return m_atomicEstimatedRobotPose.getAndSet(null);
  }

  /**
   * Allows user to select the active pipeline index
   * @param index The active pipeline index
   */
  public void setPipelineIndex(int index) {
    m_camera.setPipelineIndex(index);
  }

  /**
   * Get camera to robot transform
   * @return Camera to robot transform
   */
  public Transform3d getTransform() {
    return m_transform;
  }

  @Override
  public void close() {
    m_camera.close();
  }
}
