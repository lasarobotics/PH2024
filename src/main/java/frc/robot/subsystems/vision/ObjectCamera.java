package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.AprilTagCamera.Resolution;

/**
 * Camera that looks for rings on the ground
 */
public class ObjectCamera implements Runnable, AutoCloseable {
    PhotonCamera m_camera;
    Transform3d m_transform;

    VisionSystemSim m_visionSim;

    Pose2d m_robotPose;

    /**
     * Create VisionCamera
     *
     * @param name       Name of device
     * @param transform  Location on robot in meters
     * @param resolution Resolution used by camera
     * @param fovDiag    Diagonal FOV of camera
     */
    public ObjectCamera(String name, Transform3d transform, Resolution resolution, Rotation2d fovDiag) {
        m_camera = new PhotonCamera(name);
        m_transform = transform;

        m_visionSim = new VisionSystemSim("main");
        TargetModel targetModel = new TargetModel(0.5, 0.25);

        Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));

        VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

        m_visionSim.addVisionTargets(visionTarget);

        SimCameraProperties cameraProp = new SimCameraProperties();

        PhotonCameraSim cameraSim = new PhotonCameraSim(m_camera, cameraProp);

        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        // Add this camera to the vision system simulation with the given
        // robot-to-camera transform.
        m_visionSim.addCamera(cameraSim, robotToCamera);

        cameraSim.enableDrawWireframe(true);
    }

    public Double getDistance() {
        PhotonPipelineResult result = m_camera.getLatestResult();
        if (!result.hasTargets()) return null;

        double range = PhotonUtils.calculateDistanceToTargetMeters(
            0.5,
            2,
            -15,
            Units.degreesToRadians(result.getBestTarget().getPitch())
        );

        return range;
    }

    public Double getHeading() {
        PhotonPipelineResult result = m_camera.getLatestResult();
        if (!result.hasTargets()) return null;
        return result.getBestTarget().getYaw();
    }

    public void run(Pose2d pose) {
        m_robotPose = pose;
        m_visionSim.update(pose);
    }

    @Override
    public void close() {
        m_camera.close();
    }

    @Override
    public void run() {}
}