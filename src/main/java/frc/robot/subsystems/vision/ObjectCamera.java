package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.subsystems.vision.AprilTagCamera.Resolution;

/**
 * Camera that looks for rings on the ground
 */
public class ObjectCamera implements AutoCloseable {
  private static final double TARGET_HEIGHT_METERS = 0.0508;

  private PhotonCamera m_camera;
  private PhotonCameraSim m_cameraSim;
  private Transform3d m_transform;
  private VisionTargetSim m_targetSim;

  /**
    * Create Object Camera
    *
    * @param name       Name of device
    * @param transform  Location on robot in meters
    * @param resolution Resolution used by camera
    * @param fovDiag    Diagonal FOV of camera
    */
  public ObjectCamera(String name, Transform3d transform, Resolution resolution, Rotation2d fovDiag) {
    m_camera = new PhotonCamera(name);
    m_transform = transform;


    TargetModel targetModel = new TargetModel(0.5, 0.25);
    Pose3d targetPose = new Pose3d(Constants.Field.FIELD_LENGTH, Constants.Field.FIELD_WIDTH / 2, TARGET_HEIGHT_METERS, new Rotation3d(0, 0, Math.PI));

    m_targetSim = new VisionTargetSim(targetPose, targetModel);


    var cameraProperties = SimCameraProperties.PERFECT_90DEG();
    cameraProperties.setCalibration(resolution.width, resolution.height, fovDiag);
    m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties);

    // Enable wireframe in sim camera stream
    m_cameraSim.enableDrawWireframe(true);
  }

  /**
   * Get simulated game object target
   * @return Target sim
   */
  public VisionTargetSim getTargetSim() {
    return m_targetSim;
  }

  /**
   * Get camera sim
   * @return Simulated camera object
   */
  public PhotonCameraSim getCameraSim() {
    return m_cameraSim;
  }

  /**
    * Get distance to game object
    * @return Distance to object, empty if undetected
    */
  public Optional<Measure<Distance>> getDistance() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (!result.hasTargets()) return Optional.empty();

    double range = PhotonUtils.calculateDistanceToTargetMeters(
        m_transform.getZ(),
        TARGET_HEIGHT_METERS,
        -m_transform.getRotation().getY(),
        Units.Degrees.of(result.getBestTarget().getPitch()).in(Units.Radians)
    );

    return Optional.of(Units.Meters.of(range));
  }

  /**
    * Get yaw angle to target
    * @return Yaw angle to target, empty if undetected
    */
  public Optional<Measure<Angle>> getYaw() {
    PhotonPipelineResult result = m_camera.getLatestResult();
    if (!result.hasTargets()) return Optional.empty();
    return Optional.of(Units.Degrees.of(result.getBestTarget().getYaw()));
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