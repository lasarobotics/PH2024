// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.lasarobotics.drive.AdvancedSwerveKinematics;
import org.lasarobotics.drive.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.MAXSwerveModule;
import org.lasarobotics.drive.RotatePIDController;
import org.lasarobotics.drive.ThrottleMap;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.led.LEDStrip;
import org.lasarobotics.led.LEDStrip.Pattern;
import org.lasarobotics.led.LEDSubsystem;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    NavX2 navx;
    MAXSwerveModule lFrontModule;
    MAXSwerveModule rFrontModule;
    MAXSwerveModule lRearModule;
    MAXSwerveModule rRearModule;
    LEDStrip ledStrip;

    public Hardware(NavX2 navx,
                    MAXSwerveModule lFrontModule,
                    MAXSwerveModule rFrontModule,
                    MAXSwerveModule lRearModule,
                    MAXSwerveModule rRearModule,
                    LEDStrip ledStrip) {
      this.navx = navx;
      this.lFrontModule = lFrontModule;
      this.rFrontModule = rFrontModule;
      this.lRearModule = lRearModule;
      this.rRearModule = rRearModule;
      this.ledStrip = ledStrip;
    }
  }

  // Drive specs
  public static final double DRIVE_WHEELBASE = 0.6;
  public static final double DRIVE_TRACK_WIDTH = 0.6;
  public final double DRIVE_MAX_LINEAR_SPEED;
  public final double DRIVE_AUTO_ACCELERATION;
  public final double DRIVE_ROTATE_VELOCITY = 12 * Math.PI;
  public final double DRIVE_ROTATE_ACCELERATION = 4 * Math.PI;
  public static final double AUTO_LOCK_TIME = 3.0;


  private ThrottleMap m_throttleMap;
  private RotatePIDController m_rotatePIDController;
  private ProfiledPIDController m_autoAimPIDController;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;
  private AdvancedSwerveKinematics m_advancedKinematics;
  private HolonomicPathFollowerConfig m_pathFollowerConfig;

  private NavX2 m_navx;
  private MAXSwerveModule m_lFrontModule;
  private MAXSwerveModule m_rFrontModule;
  private MAXSwerveModule m_lRearModule;
  private MAXSwerveModule m_rRearModule;
  private LEDStrip m_ledStrip;

  private final double TOLERANCE = 1.0;
  private final double TIP_THRESHOLD = 30.0;
  private final double BALANCED_THRESHOLD = 5.0;
  private final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.03, 0.03, Math.toRadians(1));
  private final Matrix<N3, N1> VISION_STDDEV = VecBuilder.fill(0.5, 0.5, Math.toRadians(40));
  private final TrapezoidProfile.Constraints AIM_PID_CONSTRAINT = new TrapezoidProfile.Constraints(2160.0, 2160.0);

  private final String POSE_LOG_ENTRY = "/Pose";
  private final String ACTUAL_SWERVE_STATE_LOG_ENTRY = "/ActualSwerveState";
  private final String DESIRED_SWERVE_STATE_LOG_ENTRY = "/DesiredSwerveState";

  private ControlCentricity m_controlCentricity;
  private ChassisSpeeds m_desiredChassisSpeeds;
  private boolean m_isTractionControlEnabled = true;
  private Pose2d m_previousPose;
  private Rotation2d m_currentHeading;
  private PurplePathClient m_purplePathClient;
  private Field2d m_field;

  public final Command ANTI_TIP_COMMAND = new FunctionalCommand(
    () -> m_ledStrip.set(Pattern.RED_STROBE),
    () -> antiTip(),
    (interrupted) -> {
      m_ledStrip.set(Pattern.GREEN_SOLID);
      resetTurnPID();
      lock();
      stop();
      m_ledStrip.set(Pattern.TEAM_COLOR_SOLID);
    },
    this::isBalanced,
    this
  );

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param drivetrainHardware Hardware devices required by drivetrain
   * @param pidf PID constants
   * @param turnScalar Scalar for turn input (degrees)
   * @param deadband Deadband for controller input [+0.001, +0.2]
   * @param lookAhead Turn PID lookahead, in number of loops
   * @param slipRatio Traction control slip ratio [+0.01, +0.15]
   * @param throttleInputCurve Spline function characterising throttle input
   * @param turnInputCurve Spline function characterising turn input
   */
  public DriveSubsystem(Hardware drivetrainHardware, PIDConstants pidf, ControlCentricity controlCentricity,
                        double turnScalar, double deadband, double lookAhead, double slipRatio,
                        PolynomialSplineFunction throttleInputCurve, PolynomialSplineFunction turnInputCurve) {
    setSubsystem(getClass().getSimpleName());
    DRIVE_MAX_LINEAR_SPEED = drivetrainHardware.lFrontModule.getMaxLinearSpeed();
    DRIVE_AUTO_ACCELERATION = DRIVE_MAX_LINEAR_SPEED;
    this.m_navx = drivetrainHardware.navx;
    this.m_lFrontModule = drivetrainHardware.lFrontModule;
    this.m_rFrontModule = drivetrainHardware.rFrontModule;
    this.m_lRearModule = drivetrainHardware.lRearModule;
    this.m_rRearModule = drivetrainHardware.rRearModule;
    this.m_ledStrip = drivetrainHardware.ledStrip;
    this.m_controlCentricity = controlCentricity;
    this.m_throttleMap = new ThrottleMap(throttleInputCurve, deadband, DRIVE_MAX_LINEAR_SPEED);
    this.m_rotatePIDController = new RotatePIDController(turnInputCurve, pidf, turnScalar, deadband, lookAhead);
    this.m_pathFollowerConfig = new HolonomicPathFollowerConfig(
      new com.pathplanner.lib.util.PIDConstants(5.0, 0.0, -0.5),
      new com.pathplanner.lib.util.PIDConstants(5.0, 0.0, -0.1),
      DRIVE_MAX_LINEAR_SPEED,
      m_lFrontModule.getModuleCoordinate().getNorm(),
      new ReplanningConfig(),
      GlobalConstants.ROBOT_LOOP_PERIOD
    );

    // Calibrate and reset navX
    while (m_navx.isCalibrating()) stop();
    m_navx.reset();

    // Setup turn PID
    m_rotatePIDController.setTolerance(TOLERANCE);
    m_rotatePIDController.setSetpoint(getAngle());

    // Define drivetrain kinematics
    m_kinematics = new SwerveDriveKinematics(m_lFrontModule.getModuleCoordinate(),
                                             m_rFrontModule.getModuleCoordinate(),
                                             m_lRearModule.getModuleCoordinate(),
                                             m_rRearModule.getModuleCoordinate());

    // Define advanced drivetrain kinematics
    m_advancedKinematics = new AdvancedSwerveKinematics(m_lFrontModule.getModuleCoordinate(),
                                                        m_rFrontModule.getModuleCoordinate(),
                                                        m_lRearModule.getModuleCoordinate(),
                                                        m_rRearModule.getModuleCoordinate());

    // Initialise pose estimator
    m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      Rotation2d.fromDegrees(getAngle()),
      getModulePositions(),
      new Pose2d(),
      ODOMETRY_STDDEV,
      VISION_STDDEV
    );

    // Initialise chassis speeds
    m_desiredChassisSpeeds = new ChassisSpeeds();

    // Setup anti-tip command
    new Trigger(this::isTipping).onTrue(ANTI_TIP_COMMAND);

    // Register LED strip with LED subsystem
    LEDSubsystem.getInstance().add(m_ledStrip);

    // Set LED strip to team color
    m_ledStrip.set(Pattern.TEAM_COLOR_SOLID);

    // Setup auto-aim PID controller
    m_autoAimPIDController = new ProfiledPIDController(pidf.kP, 0.0, pidf.kD, AIM_PID_CONSTRAINT, pidf.period);
    m_autoAimPIDController.enableContinuousInput(-180.0, +180.0);
    m_autoAimPIDController.setTolerance(TOLERANCE);

    // Initialise other variables
    m_previousPose = new Pose2d();
    m_currentHeading = new Rotation2d();

    // Initalise PurplePathClient
    m_purplePathClient = new PurplePathClient(this);
    m_purplePathClient.disableConnectivityCheck();

    // Initialise field
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Setup path logging callback
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      if (poses.size() < 1) return;
      var trajectory = TrajectoryGenerator.generateTrajectory(
        poses,
        new TrajectoryConfig(DRIVE_MAX_LINEAR_SPEED, DRIVE_AUTO_ACCELERATION)
      );

      m_field.getObject("currentPath").setTrajectory(trajectory);
    });

    // Set VisionSubsystem pose supplier for simulation
    VisionSubsystem.getInstance().setPoseSupplier(this::getPose);

    disableTractionControl();
  }

  /**
   * Initialize hardware devices for drive subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    NavX2 navx = new NavX2(Constants.DriveHardware.NAVX_ID, GlobalConstants.ROBOT_LOOP_HZ);

    MAXSwerveModule lFrontModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.LEFT_FRONT_DRIVE_MOTOR_ID,
        Constants.DriveHardware.LEFT_FRONT_ROTATE_MOTOR_ID
      ),
      MAXSwerveModule.ModuleLocation.LeftFront,
      Constants.Drive.GEAR_RATIO,
      Constants.Drive.DRIVE_SLIP_RATIO,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      AUTO_LOCK_TIME
    );

    MAXSwerveModule rFrontModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.RIGHT_FRONT_DRIVE_MOTOR_ID,
        Constants.DriveHardware.RIGHT_FRONT_ROTATE_MOTOR_ID
      ),
      MAXSwerveModule.ModuleLocation.RightFront,
      Constants.Drive.GEAR_RATIO,
      Constants.Drive.DRIVE_SLIP_RATIO,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      AUTO_LOCK_TIME
    );

    MAXSwerveModule lRearModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.LEFT_REAR_DRIVE_MOTOR_ID,
        Constants.DriveHardware.LEFT_REAR_ROTATE_MOTOR_ID
      ),
      MAXSwerveModule.ModuleLocation.LeftRear,
      Constants.Drive.GEAR_RATIO,
      Constants.Drive.DRIVE_SLIP_RATIO,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      AUTO_LOCK_TIME
    );

    MAXSwerveModule rRearModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.RIGHT_REAR_DRIVE_MOTOR_ID,
        Constants.DriveHardware.RIGHT_REAR_ROTATE_MOTOR_ID
      ),
      MAXSwerveModule.ModuleLocation.RightRear,
      Constants.Drive.GEAR_RATIO,
      Constants.Drive.DRIVE_SLIP_RATIO,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      AUTO_LOCK_TIME
    );

    LEDStrip ledStrip = new LEDStrip(LEDStrip.initializeHardware(Constants.DriveHardware.LED_STRIP_ID));

    Hardware drivetrainHardware = new Hardware(navx, lFrontModule, rFrontModule, lRearModule, rRearModule, ledStrip);

    return drivetrainHardware;
  }

  /**
   * Set swerve modules
   * @param moduleStates Array of calculated module states
   */
  private void setSwerveModules(SwerveModuleState[] moduleStates) {
    m_lFrontModule.set(moduleStates);
    m_rFrontModule.set(moduleStates);
    m_lRearModule.set(moduleStates);
    m_rRearModule.set(moduleStates);
    Logger.recordOutput(getName() + DESIRED_SWERVE_STATE_LOG_ENTRY, moduleStates);
  }

  /**
   * Set swerve modules, automatically applying traction control
   * @param moduleStates Array of calculated module states
   * @param inertialVelocity Current inertial velocity (m/s)
   * @param rotateRate Current rotate rate (degrees/s)
   */
  private void setSwerveModules(SwerveModuleState[] moduleStates, double inertialVelocity, double rotateRate) {
    m_lFrontModule.set(moduleStates, inertialVelocity, rotateRate);
    m_rFrontModule.set(moduleStates, inertialVelocity, rotateRate);
    m_lRearModule.set(moduleStates, inertialVelocity, rotateRate);
    m_rRearModule.set(moduleStates, inertialVelocity, rotateRate);
    Logger.recordOutput(getName() + DESIRED_SWERVE_STATE_LOG_ENTRY, moduleStates);
  }

  /**
   * Drive robot and apply traction control
   * @param xRequest Desired X (forward) velocity (m/s)
   * @param yRequest Desired Y (sideways) velocity (m/s)
   * @param rotateRequest Desired rotate rate (degrees/s)
   * @param inertialVelocity Current robot inertial velocity (m/s)
   * @param rotateRate Current robot rotate rate (degrees/s)
   */
  private void drive(double xRequest, double yRequest, double rotateRequest, double inertialVelocity, double rotateRate) {
    // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
      new ChassisSpeeds(xRequest, yRequest, Math.toRadians(rotateRequest))
    );

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = m_advancedKinematics.toSwerveModuleStates(
      m_desiredChassisSpeeds,
      getPose().getRotation(),
      m_controlCentricity
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITH traction control
    setSwerveModules(moduleStates, inertialVelocity, rotateRate);
  }

  /**
   * Drive robot without traction control
   * @param xRequest Desired X (forward) velocity (m/s)
   * @param yRequest Desired Y (sideways) velocity (m/s)
   * @param rotateRequest Desired rotate rate (degrees/s)
   */
  private void drive(double xRequest, double yRequest, double rotateRequest) {
    // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
      new ChassisSpeeds(xRequest, yRequest, Math.toRadians(rotateRequest))
    );

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = m_advancedKinematics.toSwerveModuleStates(
      m_desiredChassisSpeeds,
      getPose().getRotation(),
      m_controlCentricity
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITHOUT traction control
    setSwerveModules(moduleStates);
  }

  /**
   * Reset drive encoders
   */
  private void resetEncoders() {
    m_lFrontModule.resetDriveEncoder();
    m_rFrontModule.resetDriveEncoder();
    m_lRearModule.resetDriveEncoder();
    m_rRearModule.resetDriveEncoder();
  }

  /**
   * Get current module states
   * @return Array of swerve module states
   */
  private SwerveModuleState[] getModuleStates() {
     return new SwerveModuleState[] {
      m_lFrontModule.getState(),
      m_rFrontModule.getState(),
      m_lRearModule.getState(),
      m_rRearModule.getState()
    };
  }

  /**
   * Get current module positions
   * @return Array of swerve module positions
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_lFrontModule.getPosition(),
      m_rFrontModule.getPosition(),
      m_lRearModule.getPosition(),
      m_rRearModule.getPosition()
    };
  }

  /**
   * Update robot pose
   */
  private void updatePose() {
    // Save previous pose
    m_previousPose = getPose();

    // Update pose based on odometry
    m_poseEstimator.update(getRotation2d(), getModulePositions());

    // Update current heading
    m_currentHeading = new Rotation2d(getPose().getX() - m_previousPose.getX(), getPose().getY() - m_previousPose.getY());

    // Get estimated poses from VisionSubsystem
    var visionEstimatedRobotPoses = VisionSubsystem.getInstance().getEstimatedGlobalPoses();

    // Exit if no valid vision pose estimates
    if (visionEstimatedRobotPoses.isEmpty()) return;

    // Add vision measurements to pose estimator
    for (var visionEstimatedRobotPose : visionEstimatedRobotPoses)
      m_poseEstimator.addVisionMeasurement(visionEstimatedRobotPose.estimatedPose.toPose2d(), visionEstimatedRobotPose.timestampSeconds);
  }

  /**
   * Log DriveSubsystem outputs
   */
  private void logOutputs() {
    Logger.recordOutput(getName() + POSE_LOG_ENTRY, getPose());
    Logger.recordOutput(getName() + ACTUAL_SWERVE_STATE_LOG_ENTRY, getModuleStates());
  }

  /**
   * SmartDashboard indicators
   */
  private void smartDashboard() {
    m_field.setRobotPose(getPose());
    SmartDashboard.putBoolean("TC", m_isTractionControlEnabled);
    SmartDashboard.putBoolean("PurplePath", m_purplePathClient.isConnected());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_navx.periodic();
    m_lFrontModule.periodic();
    m_rFrontModule.periodic();
    m_lRearModule.periodic();
    m_rRearModule.periodic();

    m_purplePathClient.periodic();

    if (RobotBase.isSimulation()) return;
    updatePose();
    smartDashboard();
    logOutputs();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run in simulation
    m_lFrontModule.simulationPeriodic();
    m_rFrontModule.simulationPeriodic();
    m_lRearModule.simulationPeriodic();
    m_rRearModule.simulationPeriodic();

    double angle = m_navx.getSimAngle() + Math.toDegrees(m_desiredChassisSpeeds.omegaRadiansPerSecond) * GlobalConstants.ROBOT_LOOP_PERIOD;
    m_navx.setSimAngle(angle);

    updatePose();
    smartDashboard();
    logOutputs();
  }

  /**
   * Configure AutoBuilder for PathPlannerLib
   */
  public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getChassisSpeeds, this::autoDrive, m_pathFollowerConfig, this);
  }

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param rotateRequest Desired rotate speed [-1.0, +1.0]
   */
  public void teleopPID(double xRequest, double yRequest, double rotateRequest) {
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);

    double velocityOutput = m_throttleMap.throttleLookup(moveRequest);
    double rotateOutput = m_rotatePIDController.calculate(getAngle(), getRotateRate(), rotateRequest);

    m_autoAimPIDController.calculate(getPose().getRotation().getDegrees(), getPose().getRotation().getDegrees());

    drive(velocityOutput * Math.cos(moveDirection), velocityOutput * Math.sin(moveDirection), rotateOutput, getInertialVelocity(), getRotateRate());
  }

  /**
   * Call this repeatedly to drive during autonomous
   * @param moduleStates Calculated swerve module states
   */
  public void autoDrive(ChassisSpeeds speeds) {
    // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(speeds);

    // Invert rotation
    m_desiredChassisSpeeds.omegaRadiansPerSecond = -m_desiredChassisSpeeds.omegaRadiansPerSecond;

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = m_advancedKinematics.toSwerveModuleStates(
      m_desiredChassisSpeeds,
      getPose().getRotation(),
      ControlCentricity.ROBOT_CENTRIC
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITHOUT traction control
    setSwerveModules(moduleStates);

    // Update turn PID
    m_rotatePIDController.calculate(getAngle(), getRotateRate(), 0.0);
  }

  /**
   * Start calling this repeatedly when robot is in danger of tipping over
   */
  public void antiTip() {
    // Calculate direction of tip
    double direction = Math.atan2(getRoll(), getPitch());

    // Drive to counter tipping motion
    drive(DRIVE_MAX_LINEAR_SPEED / 4 * Math.cos(direction), DRIVE_MAX_LINEAR_SPEED / 4 * Math.sin(direction), 0.0);
  }

  /**
   * Aim robot at a desired point on the field
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param point Target point
   */
  public double aimAtPoint(double xRequest, double yRequest, Translation2d point) {
    // Calculate desired robot velocity
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);
    double velocityOutput = m_throttleMap.throttleLookup(moveRequest);

    // Get current pose
    Pose2d currentPose = getPose();
    // Angle to target point
    Rotation2d targetAngle = new Rotation2d(point.getX() - currentPose.getX(), point.getY() - currentPose.getY());
    // Movement vector of robot
    Vector2D robotVector = new Vector2D(velocityOutput * m_currentHeading.getCos(), velocityOutput * m_currentHeading.getSin());
    // Aim point
    Translation2d aimPoint = point.minus(new Translation2d(robotVector.getX(), robotVector.getY()));
    // Vector from robot to target
    Vector2D targetVector = new Vector2D(currentPose.getTranslation().getDistance(point) * targetAngle.getCos(), currentPose.getTranslation().getDistance(point) * targetAngle.getSin());
    // Parallel component of robot's motion to target vector
    Vector2D parallelRobotVector = targetVector.scalarMultiply(robotVector.dotProduct(targetVector) / targetVector.getNormSq());
    // Perpendicular component of robot's motion to target vector
    Vector2D perpendicularRobotVector = robotVector.subtract(parallelRobotVector);
    // Adjust aim point using calculated vector
    Translation2d adjustedPoint = point.minus(new Translation2d(perpendicularRobotVector.getX(), perpendicularRobotVector.getY()));
    // Calculate new angle using adjusted point
    Rotation2d adjustedAngle = new Rotation2d(adjustedPoint.getX() - currentPose.getX(), adjustedPoint.getY() - currentPose.getY());
    // Calculate necessary rotate rate
    double rotateOutput = -m_autoAimPIDController.calculate(currentPose.getRotation().getDegrees(), adjustedAngle.getDegrees());

    // Log aim point
    Logger.recordOutput(getName() + "/AimPoint", new Pose2d(aimPoint, new Rotation2d()));

    // Drive robot accordingly
    drive(velocityOutput * Math.cos(moveDirection), velocityOutput * Math.sin(moveDirection), rotateOutput, getInertialVelocity(), getRotateRate());

    return currentPose.getTranslation().getDistance(aimPoint);
  }

  /**
   * Aim robot at a desired point on the field (without any strafing)
   * @param point Target point
   */
  public void aimAtPoint(Translation2d point) {
    aimAtPoint(0.0, 0.0, point);
  }

  /**
   * Go to goal pose
   * @param goal Desired goal pose
   * @param parallelCommand Command to run in parallel on final approach
   * @param endCommand Command to run after goal is reached
   * @return Command that will drive robot to the desired pose
   */
  public Command goToPose(PurplePathPose goal, Command parallelCommand, Command endCommand) {
    goal.calculateFinalApproach(getPathConstraints());
    return Commands.sequence(
      defer(() ->
        m_purplePathClient.getTrajectoryCommand(goal, parallelCommand)
        .finallyDo(() -> resetTurnPID())
      ),
      runOnce(() -> stop()),
      Commands.parallel(Commands.idle(this), endCommand)
    );
  }

  /**
   * Go to goal pose
   * @param goal Desired goal pose
   * @return Command that will drive robot to the desired pose
   */
  public Command goToPose(PurplePathPose goal) {
    return goToPose(goal, Commands.none(), Commands.none());
  }

  /**
   * Reset DriveSubsystem turn PID
   */
  public void resetTurnPID() {
    m_rotatePIDController.setSetpoint(getAngle());
    m_rotatePIDController.reset();
  }

  /**
   * Lock swerve modules
   */
  public void lock() {
    m_lFrontModule.lock();
    m_rFrontModule.lock();
    m_lRearModule.lock();
    m_rRearModule.lock();
  }

  /**
   * Stop robot
   */
  public void stop() {
    m_lFrontModule.stop();
    m_rFrontModule.stop();
    m_lRearModule.stop();
    m_rRearModule.stop();
  }

  /**
   * Toggle traction control
   */
  public void toggleTractionControl() {
    m_isTractionControlEnabled = !m_isTractionControlEnabled;
    m_lFrontModule.toggleTractionControl();
    m_rFrontModule.toggleTractionControl();
    m_lRearModule.toggleTractionControl();
    m_rRearModule.toggleTractionControl();
  }

  /**
   * Enable traction control
   */
  public void enableTractionControl() {
    m_isTractionControlEnabled = true;
    m_lFrontModule.enableTractionControl();
    m_rFrontModule.enableTractionControl();
    m_lRearModule.enableTractionControl();
    m_rRearModule.enableTractionControl();
  }

  /**
   * Disable traction control
   */
  public void disableTractionControl() {
    m_isTractionControlEnabled = false;
    m_lFrontModule.disableTractionControl();
    m_rFrontModule.disableTractionControl();
    m_lRearModule.disableTractionControl();
    m_rRearModule.disableTractionControl();
  }

  /**
   * Reset pose estimator
   * @param pose Pose to set robot to
   */
  public void resetPose(Pose2d pose) {
    resetEncoders();
    m_poseEstimator.resetPosition(
      getRotation2d(),
      getModulePositions(),
      pose
    );
  }

  /**
   * Get path follower configuration
   * @return Path follower configuration
   */
  public HolonomicPathFollowerConfig getPathFollowerConfig() {
    return m_pathFollowerConfig;
  }

  /**
   * Get constraints for path following
   * @return Path following constraints
   */
  public PathConstraints getPathConstraints() {
    return new PathConstraints(DRIVE_MAX_LINEAR_SPEED, DRIVE_AUTO_ACCELERATION, DRIVE_ROTATE_VELOCITY, DRIVE_ROTATE_ACCELERATION);
  }

  /**
   * Get robot relative speeds
   * @return Robot relative speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Get estimated robot pose
   * @return Currently estimated robot pose
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Get drivetrain kinematics
   * @return Kinematics object
   */
  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /**
   * Get whether or not robot is tipping over
   * @return True if robot is tipping
   */
  public boolean isTipping() {
    return Math.abs(getPitch()) > TIP_THRESHOLD ||
           Math.abs(getRoll()) > TIP_THRESHOLD;
  }


  /**
   * Get whether or not robot is nearly balanced
   * @return True if robot is (nearly) balanced
   */
  public boolean isBalanced() {
    return Math.abs(getPitch()) < BALANCED_THRESHOLD &&
           Math.abs(getRoll()) < BALANCED_THRESHOLD;
  }

  /**
   * Get inertial velocity of robot
   * @return Inertial velocity of robot in m/s
   */
  public double getInertialVelocity() {
    return Math.hypot(m_navx.getInputs().xVelocity, m_navx.getInputs().yVelocity);
  }

  /**
   * Get pitch of robot
   * @return Current pitch angle of robot in degrees
   */
  public double getPitch() {
    // Robot pitch axis is navX pitch axis
    return m_navx.getInputs().pitchAngle;
  }

  /**
   * Get roll of robot
   * @return Current roll angle of robot in degrees
   */
  public double getRoll() {
    // Robot roll axis is navX roll axis
    return m_navx.getInputs().rollAngle;
  }

  /**
   * Return the heading of the robot in degrees
   * @return Current heading of the robot in degrees
   */
  public double getAngle() {
    return m_navx.getInputs().yawAngle;
  }

  /**
   * Get rotate rate of robot
   * @return Current rotate rate of robot (degrees/s)
   */
  public double getRotateRate() {
    return m_navx.getInputs().yawRate;
  }

  /**
   * Return the heading of the robot as a Rotation2d.
   *
   * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the
   * top. It needs to follow the NWU axis convention.
   *
   * @return Current heading of the robot as a Rotation2d.
   */
  public Rotation2d getRotation2d() {
    return m_navx.getInputs().rotation2d;
  }

  @Override
  public void close() {
    m_navx.close();
    m_lFrontModule.close();
    m_rFrontModule.close();
    m_lRearModule.close();
    m_rRearModule.close();
  }
}
