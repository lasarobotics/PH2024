// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.concurrent.ThreadLocalRandom;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.lasarobotics.drive.AdvancedSwerveKinematics;
import org.lasarobotics.drive.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.MAXSwerveModule;
import org.lasarobotics.drive.ModuleLocation;
import org.lasarobotics.drive.RotatePIDController;
import org.lasarobotics.drive.SwervePoseEstimatorService;
import org.lasarobotics.drive.ThrottleMap;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.led.LEDStrip.Pattern;
import org.lasarobotics.led.LEDSubsystem;
import org.lasarobotics.utils.CommonTriggers;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;
import org.lasarobotics.vision.AprilTagCamera;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
    AprilTagCamera frontCamera;
    AprilTagCamera rearCamera;

    public Hardware(NavX2 navx,
                    MAXSwerveModule lFrontModule,
                    MAXSwerveModule rFrontModule,
                    MAXSwerveModule lRearModule,
                    MAXSwerveModule rRearModule,
                    AprilTagCamera frontCamera,
                    AprilTagCamera rearCamera) {
      this.navx = navx;
      this.lFrontModule = lFrontModule;
      this.rFrontModule = rFrontModule;
      this.lRearModule = lRearModule;
      this.rRearModule = rRearModule;
      this.frontCamera = frontCamera;
      this.rearCamera = rearCamera;
    }
  }

  // Drive specs
  public static final Measure<Distance> DRIVE_WHEELBASE = Units.Meters.of(0.5588);
  public static final Measure<Distance> DRIVE_TRACK_WIDTH = Units.Meters.of(0.5588);
  public static final Measure<Mass> MASS = Units.Pounds.of(110.0);
  public static final Measure<Time> AUTO_LOCK_TIME = Units.Seconds.of(3.0);
  public static final Measure<Current> DRIVE_CURRENT_LIMIT = Units.Amps.of(60.0);
  public static final Measure<Velocity<Angle>> NAVX2_YAW_DRIFT_RATE = Units.DegreesPerSecond.of(0.5 / 60);
  public static final Measure<Velocity<Angle>> DRIVE_ROTATE_VELOCITY = Units.RadiansPerSecond.of(12 * Math.PI);
  public static final Measure<Velocity<Angle>> AIM_VELOCITY_THRESHOLD = Units.DegreesPerSecond.of(5.0);
  public static final Measure<Velocity<Angle>> VISION_ANGULAR_VELOCITY_THRESHOLD = Units.DegreesPerSecond.of(720.0);
  public static final Measure<Velocity<Velocity<Angle>>> DRIVE_ROTATE_ACCELERATION = Units.RadiansPerSecond.of(4 * Math.PI).per(Units.Second);
  public final Measure<Velocity<Distance>> DRIVE_MAX_LINEAR_SPEED;
  public final Measure<Velocity<Velocity<Distance>>> DRIVE_AUTO_ACCELERATION;

  // Other settings
  private static final double TIP_THRESHOLD = 35.0;
  private static final double TOLERANCE = 1.5;
  private static final double BALANCED_THRESHOLD = 10.0;
  private static final double AIM_VELOCITY_COMPENSATION_FUDGE_FACTOR = 0.5;
  private static final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.03, 0.03, Math.toRadians(1.0));
  private static final Matrix<N3, N1> VISION_STDDEV = VecBuilder.fill(1.0, 1.0, Math.toRadians(3.0));
  private static final PIDConstants AUTO_AIM_PID = new PIDConstants(10.0, 0.0, 0.5, 0.0, 0.0, GlobalConstants.ROBOT_LOOP_PERIOD);
  private static final TrapezoidProfile.Constraints AIM_PID_CONSTRAINT = new TrapezoidProfile.Constraints(2160.0, 4320.0);

  private static final Measure<Angle> BLUE_AMP_DIRECTION = Units.Radians.of(-Math.PI / 2);
  private static final Measure<Angle> BLUE_SOURCE_DIRECTION = Units.Radians.of(-1.060 + Math.PI);

  private static final Measure<Angle> RED_AMP_DIRECTION = Units.Radians.of(-Math.PI / 2);
  private static final Measure<Angle> RED_SOURCE_DIRECTION = Units.Radians.of(-2.106 + Math.PI);

  private static Measure<Angle> m_selectedAmpDirection = BLUE_AMP_DIRECTION;
  private static Measure<Angle> m_selectedSourceDirection = BLUE_SOURCE_DIRECTION;

  // Log
  private static final String POSE_LOG_ENTRY = "/Pose";
  private static final String ACTUAL_SWERVE_STATE_LOG_ENTRY = "/ActualSwerveState";
  private static final String DESIRED_SWERVE_STATE_LOG_ENTRY = "/DesiredSwerveState";
  private static final String IS_AIMED_LOG_ENTRY = "/IsAimed";

  private final Command SET_ALLIANCE_COMMAND = Commands.runOnce(() -> {
    // Try to get alliance
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return;

    // Set alliance if available
    setAlliance(alliance.get());
  }).andThen(Commands.waitSeconds(5)).ignoringDisable(true).repeatedly();

  public final Command ANTI_TIP_COMMAND = new FunctionalCommand(
    () -> LEDSubsystem.getInstance().startOverride(Pattern.RED_STROBE),
    () -> antiTip(),
    (interrupted) -> {
      resetRotatePID();
      stop();
      lock();
      LEDSubsystem.getInstance().endOverride();
    },
    this::isBalanced,
    this
  );

  private ThrottleMap m_throttleMap;
  private RotatePIDController m_rotatePIDController;
  private ProfiledPIDController m_autoAimPIDControllerFront;
  private ProfiledPIDController m_autoAimPIDControllerBack;
  private SwerveDriveKinematics m_kinematics;
  private SwervePoseEstimatorService m_swervePoseEstimatorService;
  private AdvancedSwerveKinematics m_advancedKinematics;
  private HolonomicPathFollowerConfig m_pathFollowerConfig;

  private NavX2 m_navx;
  private MAXSwerveModule m_lFrontModule;
  private MAXSwerveModule m_rFrontModule;
  private MAXSwerveModule m_lRearModule;
  private MAXSwerveModule m_rRearModule;


  private ControlCentricity m_controlCentricity;
  private ChassisSpeeds m_desiredChassisSpeeds;
  private Rotation2d m_allianceCorrection;
  private Pose2d m_previousPose;
  private Rotation2d m_currentHeading;
  private PurplePathClient m_purplePathClient;
  private Field2d m_field;
  private Alliance m_currentAlliance;

  private boolean m_isTractionControlEnabled = true;
  private boolean m_autoAimFront = false;
  private boolean m_autoAimBack = false;

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param drivetrainHardware Hardware devices required by drivetrain
   * @param pidf PID constants
   * @param controlCentricity Control centricity
   * @param throttleInputCurve Spline function characterising throttle input
   * @param turnInputCurve Spline function characterising turn input
   * @param turnScalar Scalar for turn input (degrees)
   * @param deadband Deadband for controller input [+0.001, +0.2]
   * @param lookAhead Rotate PID lookahead, in number of loops
   */
  public DriveSubsystem(Hardware drivetrainHardware, PIDConstants pidf, ControlCentricity controlCentricity,
                        PolynomialSplineFunction throttleInputCurve, PolynomialSplineFunction turnInputCurve,
                        double turnScalar, double deadband, double lookAhead) {
    setSubsystem(getClass().getSimpleName());
    DRIVE_MAX_LINEAR_SPEED = drivetrainHardware.lFrontModule.getMaxLinearSpeed();
    DRIVE_AUTO_ACCELERATION = DRIVE_MAX_LINEAR_SPEED.per(Units.Second).minus(Units.MetersPerSecondPerSecond.of(1.0));
    this.m_navx = drivetrainHardware.navx;
    this.m_lFrontModule = drivetrainHardware.lFrontModule;
    this.m_rFrontModule = drivetrainHardware.rFrontModule;
    this.m_lRearModule = drivetrainHardware.lRearModule;
    this.m_rRearModule = drivetrainHardware.rRearModule;
    this.m_controlCentricity = controlCentricity;
    this.m_throttleMap = new ThrottleMap(throttleInputCurve, DRIVE_MAX_LINEAR_SPEED, deadband);
    this.m_rotatePIDController = new RotatePIDController(turnInputCurve, pidf, turnScalar, deadband, lookAhead);
    this.m_pathFollowerConfig = new HolonomicPathFollowerConfig(
      new com.pathplanner.lib.util.PIDConstants(3.1, 0.0, 0.0),
      new com.pathplanner.lib.util.PIDConstants(5.0, 0.0, 0.1),
      DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond),
      m_lFrontModule.getModuleCoordinate().getNorm(),
      new ReplanningConfig(),
      GlobalConstants.ROBOT_LOOP_PERIOD
    );
    this.m_currentAlliance = Alliance.Blue;
    this.m_allianceCorrection = GlobalConstants.ROTATION_ZERO;

    // Calibrate and reset navX
    while (m_navx.isCalibrating()) stop();
    m_navx.reset();

    // Setup rotate PID
    m_rotatePIDController.setTolerance(TOLERANCE);
    m_rotatePIDController.setSetpoint(getAngle().in(Units.Degrees));

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
    m_swervePoseEstimatorService = new SwervePoseEstimatorService(
      ODOMETRY_STDDEV,
      m_navx,
      m_lFrontModule,
      m_rFrontModule,
      m_lRearModule,
      m_rRearModule
    );
    m_swervePoseEstimatorService.addAprilTagCamera(drivetrainHardware.frontCamera, drivetrainHardware.rearCamera);
    m_swervePoseEstimatorService.start();

    // Initialise chassis speeds
    m_desiredChassisSpeeds = new ChassisSpeeds();

    // Setup anti-tip command
    new Trigger(this::isTipping).whileTrue(ANTI_TIP_COMMAND);

    // Setup auto-aim PID controller
    m_autoAimPIDControllerFront = new ProfiledPIDController(AUTO_AIM_PID.kP, 0.0, AUTO_AIM_PID.kD, AIM_PID_CONSTRAINT, AUTO_AIM_PID.period);
    m_autoAimPIDControllerFront.enableContinuousInput(-180.0, +180.0);
    m_autoAimPIDControllerFront.setTolerance(TOLERANCE);
    m_autoAimPIDControllerFront.setIZone(AUTO_AIM_PID.kIZone);
    m_autoAimPIDControllerBack = new ProfiledPIDController(AUTO_AIM_PID.kP, 0.0, AUTO_AIM_PID.kD, AIM_PID_CONSTRAINT, AUTO_AIM_PID.period);
    m_autoAimPIDControllerBack.enableContinuousInput(-180.0, +180.0);
    m_autoAimPIDControllerBack.setTolerance(TOLERANCE);
    m_autoAimPIDControllerBack.setIZone(AUTO_AIM_PID.kIZone);

    // Initialise other variables
    m_previousPose = new Pose2d();
    m_currentHeading = new Rotation2d();

    // Initalise PurplePathClient
    m_purplePathClient = new PurplePathClient(this);

    // Set alliance triggers
    CommonTriggers.isDSAttached().onTrue(SET_ALLIANCE_COMMAND);
    RobotModeTriggers.disabled().whileTrue(SET_ALLIANCE_COMMAND);

    // Initialise field
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Setup path logging callback
    PathPlannerLogging.setLogActivePathCallback(poses -> {
      if (poses.size() < 1) return;
      var trajectory = TrajectoryGenerator.generateTrajectory(
        poses,
        new TrajectoryConfig(DRIVE_MAX_LINEAR_SPEED, DRIVE_AUTO_ACCELERATION)
      );
      m_field.getObject("currentPath").setTrajectory(trajectory);
    });
  }

  /**
   * Initialize hardware devices for drive subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    NavX2 navx = new NavX2(Constants.DriveHardware.NAVX_ID, GlobalConstants.ROBOT_LOOP_HZ * 2);

    MAXSwerveModule lFrontModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.LEFT_FRONT_DRIVE_MOTOR_ID,
        Constants.DriveHardware.LEFT_FRONT_ROTATE_MOTOR_ID,
        MotorKind.NEO_VORTEX
      ),
      ModuleLocation.LeftFront,
      Constants.Drive.GEAR_RATIO,
      Constants.Drive.DRIVE_WHEEL,
      Constants.Drive.DRIVE_SLIP_RATIO,
      MASS,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      AUTO_LOCK_TIME,
      DRIVE_CURRENT_LIMIT
    );

    MAXSwerveModule rFrontModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.RIGHT_FRONT_DRIVE_MOTOR_ID,
        Constants.DriveHardware.RIGHT_FRONT_ROTATE_MOTOR_ID,
        MotorKind.NEO_VORTEX
      ),
      ModuleLocation.RightFront,
      Constants.Drive.GEAR_RATIO,
      Constants.Drive.DRIVE_WHEEL,
      Constants.Drive.DRIVE_SLIP_RATIO,
      MASS,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      AUTO_LOCK_TIME,
      DRIVE_CURRENT_LIMIT
    );

    MAXSwerveModule lRearModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.LEFT_REAR_DRIVE_MOTOR_ID,
        Constants.DriveHardware.LEFT_REAR_ROTATE_MOTOR_ID,
        MotorKind.NEO_VORTEX
      ),
      ModuleLocation.LeftRear,
      Constants.Drive.GEAR_RATIO,
      Constants.Drive.DRIVE_WHEEL,
      Constants.Drive.DRIVE_SLIP_RATIO,
      MASS,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      AUTO_LOCK_TIME,
      DRIVE_CURRENT_LIMIT
    );

    MAXSwerveModule rRearModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.RIGHT_REAR_DRIVE_MOTOR_ID,
        Constants.DriveHardware.RIGHT_REAR_ROTATE_MOTOR_ID,
        MotorKind.NEO_VORTEX
      ),
      ModuleLocation.RightRear,
      Constants.Drive.GEAR_RATIO,
      Constants.Drive.DRIVE_WHEEL,
      Constants.Drive.DRIVE_SLIP_RATIO,
      MASS,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      AUTO_LOCK_TIME,
      DRIVE_CURRENT_LIMIT
    );

    AprilTagCamera frontCamera = new AprilTagCamera(
      Constants.VisionHardware.CAMERA_A_NAME,
      Constants.VisionHardware.CAMERA_A_LOCATION,
      Constants.VisionHardware.CAMERA_A_RESOLUTION,
      Constants.VisionHardware.CAMERA_A_FOV,
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
    );

    AprilTagCamera rearCamera = new AprilTagCamera(
      Constants.VisionHardware.CAMERA_B_NAME,
      Constants.VisionHardware.CAMERA_B_LOCATION,
      Constants.VisionHardware.CAMERA_B_RESOLUTION,
      Constants.VisionHardware.CAMERA_B_FOV,
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
    );

    Hardware drivetrainHardware = new Hardware(navx, lFrontModule, rFrontModule, lRearModule, rRearModule, frontCamera, rearCamera);

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
   * @param inertialVelocity Current inertial velocity
   * @param rotateRate Current robot rotate rate
   */
  private void setSwerveModules(SwerveModuleState[] moduleStates, Measure<Velocity<Distance>> inertialVelocity, Measure<Velocity<Angle>> rotateRate) {
    m_lFrontModule.set(moduleStates, inertialVelocity, rotateRate);
    m_rFrontModule.set(moduleStates, inertialVelocity, rotateRate);
    m_lRearModule.set(moduleStates, inertialVelocity, rotateRate);
    m_rRearModule.set(moduleStates, inertialVelocity, rotateRate);
    Logger.recordOutput(getName() + DESIRED_SWERVE_STATE_LOG_ENTRY, moduleStates);
  }

  /**
   * Drive robot and apply traction control
   * @param xRequest Desired X (forward) velocity
   * @param yRequest Desired Y (sideways) velocity
   * @param rotateRequest Desired rotate rate
   * @param inertialVelocity Current robot inertial velocity
   */
  private void drive(ControlCentricity controlCentricity,
                     Measure<Velocity<Distance>> xRequest,
                     Measure<Velocity<Distance>> yRequest,
                     Measure<Velocity<Angle>> rotateRequest,
                     Measure<Velocity<Distance>> inertialVelocity,
                     Measure<Velocity<Angle>> rotateRate) {
    // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
      new ChassisSpeeds(xRequest, yRequest, rotateRequest)
    );

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = m_advancedKinematics.toSwerveModuleStates(
      m_desiredChassisSpeeds,
      getPose().getRotation().plus(m_allianceCorrection),
      controlCentricity
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITH traction control
    setSwerveModules(moduleStates, inertialVelocity, rotateRate);
  }

  /**
   * Drive robot without traction control
   * @param xRequest Desired X (forward) velocity
   * @param yRequest Desired Y (sideways) velocity
   * @param rotateRequest Desired rotate rate
   */
  private void drive(ControlCentricity controlCentricity,
                     Measure<Velocity<Distance>> xRequest,
                     Measure<Velocity<Distance>> yRequest,
                     Measure<Velocity<Angle>> rotateRequest) {
    // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
      new ChassisSpeeds(xRequest, yRequest, rotateRequest)
    );

    // Convert speeds to module states, correcting for 2nd order kinematics
    SwerveModuleState[] moduleStates = m_advancedKinematics.toSwerveModuleStates(
      m_desiredChassisSpeeds,
      getPose().getRotation().plus(m_allianceCorrection),
      controlCentricity
    );

    // Desaturate drive speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

    // Set modules to calculated states, WITHOUT traction control
    setSwerveModules(moduleStates);
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
    //m_poseEstimator.update(getRotation2d(), getModulePositions());

    // Update current heading
    m_currentHeading = new Rotation2d(getPose().getX() - m_previousPose.getX(), getPose().getY() - m_previousPose.getY());

    // Get estimated poses from VisionSubsystem
    var apriltagCameraResults = VisionSubsystem.getInstance().getEstimatedGlobalPoses();

    // Exit if no valid vision pose estimates
    if (apriltagCameraResults.isEmpty()) return;

    // Exit if robot is spinning too fast
    if (getRotateRate().gt(VISION_ANGULAR_VELOCITY_THRESHOLD)) return;

    // Add vision measurements to pose estimator
    // for (var result : apriltagCameraResults) {
    //   m_poseEstimator.addVisionMeasurement(
    //     result.estimatedRobotPose.estimatedPose.toPose2d(),
    //     result.estimatedRobotPose.timestampSeconds,
    //     result.visionMeasurementStdDevs
    //   );
    // }
  }

  /**
   * Log DriveSubsystem outputs
   */
  private void logOutputs() {
    Logger.recordOutput(getName() + POSE_LOG_ENTRY, getPose());
    Logger.recordOutput(getName() + ACTUAL_SWERVE_STATE_LOG_ENTRY, getModuleStates());
    Logger.recordOutput(getName() + IS_AIMED_LOG_ENTRY, isAimed());
  }

  /**
   * SmartDashboard indicators
   */
  private void smartDashboard() {
    m_field.setRobotPose(getPose());
    SmartDashboard.putBoolean("TC", m_isTractionControlEnabled);
    SmartDashboard.putBoolean("PurplePath", m_purplePathClient.isConnected());
    SmartDashboard.putBoolean("FC", m_controlCentricity.equals(ControlCentricity.FIELD_CENTRIC));
    SmartDashboard.putString("Alliance", m_currentAlliance.name());
  }

  /**
   * Start calling this repeatedly when robot is in danger of tipping over
   */
  private void antiTip() {
    // Calculate direction of tip
    double direction = Math.atan2(getRoll().in(Units.Degrees), getPitch().in(Units.Degrees));

    // Drive to counter tipping motion
    drive(
      ControlCentricity.ROBOT_CENTRIC,
      DRIVE_MAX_LINEAR_SPEED.divide(4).times(Math.cos(direction)),
      DRIVE_MAX_LINEAR_SPEED.divide(4).times(Math.sin(direction)),
      Units.DegreesPerSecond.of(0.0)
    );
  }

  /**
   * Aim robot at a desired point on the field
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param rotateRequest Desired rotate speed (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param point Target point, pass in null to signify invalid point
   * @param boolean True to point back of robot to target
   * @param velocityCorrection True to compensate for robot's own velocity
   */
  private void aimAtPoint(ControlCentricity controlCentricity, double xRequest, double yRequest, double rotateRequest, Translation2d point, boolean reversed, boolean velocityCorrection) {
    // Calculate desired robot velocity
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);
    var velocityOutput = m_throttleMap.throttleLookup(moveRequest).negate();

    // Drive normally and return if invalid point
    if (point == null) {
      var rotateOutput = m_rotatePIDController.calculate(getAngle(), getRotateRate(), rotateRequest).negate();
      drive(
        m_controlCentricity,
        velocityOutput.times(Math.cos(moveDirection)),
        velocityOutput.times(Math.sin(moveDirection)),
        rotateOutput,
        getInertialVelocity(),
        getRotateRate()
      );
      return;
    }

    // Mark PID controller being used
    if (reversed) {
      m_autoAimFront = false;
      m_autoAimBack = true;
    } else {
      m_autoAimFront = true;
      m_autoAimBack = false;
    }

    // Get current pose
    Pose2d currentPose = getPose();
    // Angle to target point
    Rotation2d targetAngle = new Rotation2d(point.getX() - currentPose.getX(), point.getY() - currentPose.getY());
    // Movement vector of robot
    Vector2D robotVector = new Vector2D(velocityOutput.times(m_currentHeading.getCos()).in(Units.MetersPerSecond), velocityOutput.times(m_currentHeading.getSin()).in(Units.MetersPerSecond));
    // Aim point
    Translation2d aimPoint = point.minus(new Translation2d(robotVector.getX(), robotVector.getY()));
    // Vector from robot to target
    Vector2D targetVector = new Vector2D(currentPose.getTranslation().getDistance(point) * targetAngle.getCos(), currentPose.getTranslation().getDistance(point) * targetAngle.getSin());
    // Parallel component of robot's motion to target vector
    Vector2D parallelRobotVector = targetVector.scalarMultiply(robotVector.dotProduct(targetVector) / targetVector.getNormSq());
    // Perpendicular component of robot's motion to target vector
    Vector2D perpendicularRobotVector = robotVector.subtract(parallelRobotVector).scalarMultiply(velocityCorrection ? AIM_VELOCITY_COMPENSATION_FUDGE_FACTOR : 0.0);
    // Adjust aim point using calculated vector
    Translation2d adjustedPoint = point.minus(new Translation2d(perpendicularRobotVector.getX(), perpendicularRobotVector.getY()));
    // Calculate new angle using adjusted point
    Rotation2d adjustedAngle = new Rotation2d(adjustedPoint.getX() - currentPose.getX(), adjustedPoint.getY() - currentPose.getY());
    // Calculate necessary rotate rate
    var rotateOutput = reversed
      ? Units.DegreesPerSecond.of(m_autoAimPIDControllerBack.calculate(currentPose.getRotation().plus(GlobalConstants.ROTATION_PI).getDegrees(), adjustedAngle.getDegrees()))
      : Units.DegreesPerSecond.of(m_autoAimPIDControllerFront.calculate(currentPose.getRotation().getDegrees(), adjustedAngle.getDegrees()));

    // Log aim point
    double aimError = currentPose.getRotation().getDegrees() - adjustedAngle.getDegrees();
    Logger.recordOutput(getName() + "/AimPoint", new Pose2d(aimPoint, new Rotation2d()));
    Logger.recordOutput(getName() + "/AimError", Math.copySign(((180 - Math.abs(aimError)) % 180), (aimError)));

    // Drive robot accordingly
     drive(
      m_controlCentricity,
      velocityOutput.times(Math.cos(moveDirection)),
      velocityOutput.times(Math.sin(moveDirection)),
      rotateOutput,
      getInertialVelocity(),
      getRotateRate()
    );
  }

  /**
   * Rotates the robot to the nearest cardinal direction while preserving strafing
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   */
  private void snapToImportantDirection(double xRequest, double yRequest) {
    // Calculate desired robot velocity
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);
    var velocityOutput = m_throttleMap.throttleLookup(moveRequest).negate();

    double sourceDistance = getPose().getTranslation().getDistance(Constants.Field.SOURCE.getGoalPose().getTranslation());

    Rotation2d currentRotation = getPose().getRotation();

    double desiredAngle;
    if (sourceDistance < 2){
      desiredAngle = m_selectedSourceDirection.in(Units.Degrees);
    } else desiredAngle = m_selectedAmpDirection.in(Units.Degrees);

    var rotateOutput = Units.DegreesPerSecond.of(m_autoAimPIDControllerFront.calculate(currentRotation.getDegrees(), desiredAngle));

    // Drive with the pose to the snapped cardinal direction
    drive(
      m_controlCentricity,
      velocityOutput.times(Math.cos(moveDirection)),
      velocityOutput.times(Math.sin(moveDirection)),
      rotateOutput,
      getInertialVelocity(),
      getRotateRate()
    );
  }

  /**
   * Rotates the robot to the nearest cardinal direction while preserving strafing
   * @param xRequestSupplier X axis speed supplier
   * @param yRequestSupplier Y axis speed supplier
   * @return Command to snap to the nearest cardinal direction
   */
  public Command snapToImportantDirectionCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier) {
    return runEnd(
      () -> snapToImportantDirection(xRequestSupplier.getAsDouble(), yRequestSupplier.getAsDouble()),
      () -> resetRotatePID()
    );
  }

  /**
   *
   * @param xRequest
   * @param yRequest
   * @param rotateRequest
   */
  private void autoDefense(double xRequest, double yRequest, double rotateRequest) {
    Optional<Measure<Angle>> objectYaw = VisionSubsystem.getInstance().getObjectHeading();
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);
    var velocityOutput = m_throttleMap.throttleLookup(moveRequest).negate();
    var rotateOutput = m_rotatePIDController.calculate(getAngle(), getRotateRate(), rotateRequest).negate();

    if (objectYaw.isEmpty()) {
      drive(
        m_controlCentricity,
        velocityOutput.times(Math.cos(moveDirection)),
        velocityOutput.times(Math.sin(moveDirection)),
        rotateOutput,
        getInertialVelocity(),
        getRotateRate()
      );
      return;
    }

    moveRequest = Math.hypot(xRequest, 0.0);
    moveDirection = Math.atan2(0.0, xRequest);
    velocityOutput = m_throttleMap.throttleLookup(moveRequest);
    System.out.println(rotateOutput);
    drive(
      ControlCentricity.ROBOT_CENTRIC,
      velocityOutput,
      DRIVE_MAX_LINEAR_SPEED.times(objectYaw.get().in(Units.Degrees) / Constants.VisionHardware.CAMERA_OBJECT_FOV.getDegrees()),
      rotateOutput,
      getInertialVelocity(),
      getRotateRate()
    );
  }

  /**
   * Aim robot by given angle
   * @param angle Desired angle in degrees
   */
  private void aimAtAngle(double angle) {
    double rotateOutput = m_rotatePIDController.calculate(getAngle().in(Units.Degrees), getAngle().in(Units.Degrees) + angle);

    drive(
      m_controlCentricity,
      Units.MetersPerSecond.of(0),
      Units.MetersPerSecond.of(0),
      Units.DegreesPerSecond.of(rotateOutput),
      getInertialVelocity(),
      getRotateRate()
    );
  }

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param rotateRequest Desired rotate speed [-1.0, +1.0]
   */
  private void teleopPID(double xRequest, double yRequest, double rotateRequest) {
    // Calculate move request and direction
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);

    // Get throttle and rotate output
    var velocityOutput = m_throttleMap.throttleLookup(moveRequest).negate();
    var rotateOutput = m_rotatePIDController.calculate(getAngle(), getRotateRate(), rotateRequest).negate();

    // Update auto-aim controllers
    m_autoAimPIDControllerFront.calculate(
      getPose().getRotation().getDegrees(),
      getPose().getRotation().getDegrees()
    );
    m_autoAimPIDControllerBack.calculate(
      getPose().getRotation().plus(GlobalConstants.ROTATION_PI).getDegrees(),
      getPose().getRotation().plus(GlobalConstants.ROTATION_PI).getDegrees()
    );
    m_autoAimFront = false;
    m_autoAimBack = false;

    // Drive robot
    drive(
      m_controlCentricity,
      velocityOutput.times(Math.cos(moveDirection)),
      velocityOutput.times(Math.sin(moveDirection)),
      rotateOutput,
      getInertialVelocity(),
      getRotateRate()
    );
  }

  /**
   * Lock swerve modules
   */
  private void lock() {
    m_lFrontModule.lock();
    m_rFrontModule.lock();
    m_lRearModule.lock();
    m_rRearModule.lock();
  }

  /**
   * Stop robot
   */
  private void stop() {
    m_lFrontModule.stop();
    m_rFrontModule.stop();
    m_lRearModule.stop();
    m_rRearModule.stop();
  }

  /**
   * Toggle traction control
   */
  private void toggleTractionControl() {
    m_isTractionControlEnabled = !m_isTractionControlEnabled;
    m_lFrontModule.toggleTractionControl();
    m_rFrontModule.toggleTractionControl();
    m_lRearModule.toggleTractionControl();
    m_rRearModule.toggleTractionControl();
  }

  /**
   * Enable traction control
   */
  private void enableTractionControl() {
    m_isTractionControlEnabled = true;
    m_lFrontModule.enableTractionControl();
    m_rFrontModule.enableTractionControl();
    m_lRearModule.enableTractionControl();
    m_rRearModule.enableTractionControl();
  }

  /**
   * Disable traction control
   */
  private void disableTractionControl() {
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
  private void resetPose(Pose2d pose) {
    m_swervePoseEstimatorService.resetPose(pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Save previous pose
    m_previousPose = getPose();

    // Update current heading
    m_currentHeading = new Rotation2d(getPose().getX() - m_previousPose.getX(), getPose().getY() - m_previousPose.getY());

    if (RobotBase.isSimulation()) return;
    //updatePose();
    smartDashboard();
    logOutputs();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run in simulation
    double randomNoise = ThreadLocalRandom.current().nextDouble(0.9, 1.0);
    m_navx.getInputs().xVelocity = Units.MetersPerSecond.of(m_desiredChassisSpeeds.vxMetersPerSecond * randomNoise);
    m_navx.getInputs().yVelocity = Units.MetersPerSecond.of(m_desiredChassisSpeeds.vyMetersPerSecond * randomNoise);
    m_navx.getInputs().yawRate = Units.RadiansPerSecond.of(m_desiredChassisSpeeds.omegaRadiansPerSecond * randomNoise);

    int yawDriftDirection = ThreadLocalRandom.current().nextDouble(1.0) < 0.5 ? -1 : +1;
    double angle = m_navx.getSimAngle() - Math.toDegrees(m_desiredChassisSpeeds.omegaRadiansPerSecond * randomNoise) * GlobalConstants.ROBOT_LOOP_PERIOD
                   + (NAVX2_YAW_DRIFT_RATE.in(Units.DegreesPerSecond) * GlobalConstants.ROBOT_LOOP_PERIOD * yawDriftDirection);
    m_navx.setSimAngle(angle);

    //updatePose();
    smartDashboard();
    logOutputs();
  }

  /**
   * Configure ber auto builder
   */
  public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getChassisSpeeds,
      this::autoDrive,
      m_pathFollowerConfig,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) return alliance.get() == DriverStation.Alliance.Red;
        return false;
      },
      this
    );
  }

  /**
   * Set alliance
   * <p>
   * Must be set to correct for field oriented drive
   * @param alliance alliance
   */

  public void setAlliance(Alliance alliance) {
    m_currentAlliance = alliance;
    m_allianceCorrection = m_currentAlliance.equals(Alliance.Red) ? GlobalConstants.ROTATION_PI : GlobalConstants.ROTATION_ZERO;
    if (m_currentAlliance.equals(Alliance.Red)){
      m_selectedAmpDirection = RED_AMP_DIRECTION;
      m_selectedSourceDirection = RED_SOURCE_DIRECTION;
    } else {
      m_selectedAmpDirection = BLUE_AMP_DIRECTION;
      m_selectedSourceDirection = BLUE_SOURCE_DIRECTION;
    }
  }

  public Alliance getAlliance() {
    return m_currentAlliance;
  }

  /**
   * Call this repeatedly to drive during autonomous
   * @param moduleStates Calculated swerve module states
   */
  public void autoDrive(ChassisSpeeds speeds) {
    // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(speeds);

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

    // Update auto-aim controllers
    m_autoAimPIDControllerFront.calculate(
      getPose().getRotation().getDegrees(),
      getPose().getRotation().getDegrees()
    );
    m_autoAimPIDControllerBack.calculate(
      getPose().getRotation().plus(GlobalConstants.ROTATION_PI).getDegrees(),
      getPose().getRotation().plus(GlobalConstants.ROTATION_PI).getDegrees()
    );
  }

  /**
   * Toggles between field centric and robot centric drive control
   */
  private void toggleControlCentricity() {
    if (m_controlCentricity == ControlCentricity.FIELD_CENTRIC) {
      this.m_controlCentricity = ControlCentricity.ROBOT_CENTRIC;
    } else {
      this.m_controlCentricity = ControlCentricity.FIELD_CENTRIC;
    }
  }

  /**
   * Aim robot at desired point on the field, while strafing
   * @param xRequestSupplier X axis speed supplier [-1.0, +1.0]
   * @param yRequestSupplier Y axis speed supplier [-1.0, +1.0]
   * @param rotateRequestSupplier Rotate speed supplier (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param pointSupplier Desired point supplier
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim at point while strafing
   */
  public Command aimAtPointCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                   Supplier<Translation2d> pointSupplier, boolean reversed, boolean velocityCorrection) {
    return runEnd(
      () -> aimAtPoint(
        m_controlCentricity,
        xRequestSupplier.getAsDouble(),
        yRequestSupplier.getAsDouble(),
        rotateRequestSupplier.getAsDouble(),
        pointSupplier.get(),
        reversed,
        velocityCorrection
      ),
      () -> resetRotatePID()
    );
  }

  /**
   * Aim robot at desired point on the field, while strafing
   * @param xRequestSupplier X axis speed supplier [-1.0, +1.0]
   * @param yRequestSupplier Y axis speed supplier [-1.0, +1.0]
   * @param rotateRequestSupplier Rotate speed supplier (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param point Desired point
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim at point while strafing
   */
  public Command aimAtPointCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                   Translation2d point, boolean reversed, boolean velocityCorrection) {
    return aimAtPointCommand(xRequestSupplier, yRequestSupplier, rotateRequestSupplier, () -> point, reversed, velocityCorrection);
  }

  /**
   * Aim robot at desired point on the field
   * @param point Desired point
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim robot at point while strafing
   */
  public Command aimAtPointCommand(Translation2d point, boolean reversed, boolean velocityCorrection) {
    return aimAtPointCommand(() -> 0.0, () -> 0.0, () -> 0.0, () -> point, reversed, velocityCorrection);
  }

  /**
   * Change robot aim by desired angle
   * @param angleRequestSupplier
   * @return Command that aims robot
   */
  public Command aimAtAngleCommand(DoubleSupplier angleRequestSupplier) {
    return run(() -> aimAtAngle(angleRequestSupplier.getAsDouble()));
  }

  /**
   *
   * @param xRequestSupplier
   * @param yRequestSupplier
   * @param rotateRequestSupplier
   * @return
   */
  public Command autoDefenseCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier) {
    return runEnd(
      () -> autoDefense(xRequestSupplier.getAsDouble(), yRequestSupplier.getAsDouble(), rotateRequestSupplier.getAsDouble()),
      () -> resetRotatePID()
    );
  }

  /**
   * Drive the robot
   * @param xRequestSupplier X axis speed supplier
   * @param yRequestSupplier Y axis speed supplier
   * @param rotateRequestSupplier Rotate speed supplier
   * @return Command that will drive robot
   */
  public Command driveCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier) {
    return run(() -> teleopPID(xRequestSupplier.getAsDouble(), yRequestSupplier.getAsDouble(), rotateRequestSupplier.getAsDouble()));
  }

  /**
   * Lock swerve modules
   * @return Command to lock swerve modules
   */
  public Command lockCommand() {
    return runOnce(() -> lock());
  }

  /**
   * Stop robot
   * @return Command to stop robot
   */
  public Command stopCommand() {
    return runOnce(() -> {
      stop();
      resetRotatePID();
    });
  }

  /**
   * Toggle traction control
   * @return Command to toggle traction control
   */
  public Command toggleTractionControlCommand() {
    return runOnce(() -> toggleTractionControl());
  }

  /**
   * Toggles between field and robot oriented drive control
   * @return Command to toggle control centricity between robot and field centric drive control
   */
  public Command toggleCentricityCommand() {
    return runOnce(() -> toggleControlCentricity());
  }

  /**
   * Enable traction control
   * @return Command to enable traction control
   */
  public Command enableTractionControlCommand() {
    return runOnce(() -> enableTractionControl());
  }

  /**
   * Disable traction control
   * @return Command to disable traction control
   */
  public Command disableTractionControlCommand() {
    return runOnce(() -> disableTractionControl());
  }

  /**
   * Reset pose estimator
   * @param poseSupplier Pose supplier
   * @return Command to reset pose
   */
  public Command resetPoseCommand(Supplier<Pose2d> poseSupplier) {
    return runOnce(() -> resetPose(poseSupplier.get()));
  }

  /**
   * Go to goal pose
   * @param goal Desired goal pose
   * @param parallelCommand Command to run in parallel on final approach
   * @param endCommand Command to run after goal is reached
   * @return Command that will drive robot to the desired pose
   */
  public Command goToPoseCommand(PurplePathPose goal, Command parallelCommand, Command endCommand) {
    goal.calculateFinalApproach(getPathConstraints());
    return Commands.sequence(
      defer(() -> m_purplePathClient.getTrajectoryCommand(goal, parallelCommand).finallyDo(() -> resetRotatePID())),
      stopCommand(),
      Commands.parallel(driveCommand(() -> 0.0, () -> 0.0, () -> 0.0), endCommand)
    );
  }

  /**
   * @return Command to aim a point on the field in robot centric mode
   */
  public Command aimAtPointRobotCentric(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                        Supplier<Translation2d> pointSupplier, boolean reversed, boolean velocityCorrection) {
    return runEnd(() ->
      aimAtPoint(
        ControlCentricity.ROBOT_CENTRIC,
        xRequestSupplier.getAsDouble(),
        yRequestSupplier.getAsDouble(),
        rotateRequestSupplier.getAsDouble(),
        pointSupplier.get(),
        reversed,
        velocityCorrection
      ),
      () -> resetRotatePID()
    );

  }

  /**
   * Go to goal pose
   * @param goal Desired goal pose
   * @return Command that will drive robot to the desired pose
   */
  public Command goToPoseCommand(PurplePathPose goal) {
    return goToPoseCommand(goal, Commands.none(), Commands.none());
  }

  /**
   * Reset DriveSubsystem turn PID
   */
  public void resetRotatePID() {
    m_rotatePIDController.setSetpoint(getAngle().in(Units.Degrees));
    m_rotatePIDController.reset();
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
    return new PathConstraints(
      3.0,
      1.0,
      DRIVE_ROTATE_VELOCITY.in(Units.RadiansPerSecond),
      DRIVE_ROTATE_ACCELERATION.magnitude()
    );
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
    return m_swervePoseEstimatorService.getPose();
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
    return Math.abs(getPitch().in(Units.Degrees)) > TIP_THRESHOLD ||
           Math.abs(getRoll().in(Units.Degrees)) > TIP_THRESHOLD;
  }


  /**
   * Get whether or not robot is nearly balanced
   * @return True if robot is (nearly) balanced
   */
  public boolean isBalanced() {
    return Math.abs(getPitch().in(Units.Degrees)) < BALANCED_THRESHOLD &&
           Math.abs(getRoll().in(Units.Degrees)) < BALANCED_THRESHOLD;
  }

  /**
   * Get if robot is aimed at desired target
   * @return True if aimed
   */
  public boolean isAimed() {
    return (
      m_autoAimPIDControllerFront.atGoal() & m_autoAimFront
      |
      m_autoAimPIDControllerBack.atGoal() & m_autoAimBack
    ) && getRotateRate().lt(AIM_VELOCITY_THRESHOLD);
  }

  /**
   * Get inertial velocity of robot
   * @return Inertial velocity of robot in m/s
   */
  public Measure<Velocity<Distance>> getInertialVelocity() {
    return Units.MetersPerSecond.of(
      Math.hypot(m_navx.getInputs().xVelocity.in(Units.MetersPerSecond), m_navx.getInputs().yVelocity.in(Units.MetersPerSecond))
    );
  }

  /**
   * Get pitch of robot
   * @return Current pitch angle of robot in degrees
   */
  public Measure<Angle> getPitch() {
    // Robot pitch axis is navX pitch axis
    return m_navx.getInputs().pitchAngle;
  }

  /**
   * Get roll of robot
   * @return Current roll angle of robot in degrees
   */
  public Measure<Angle> getRoll() {
    // Robot roll axis is navX roll axis
    return m_navx.getInputs().rollAngle;
  }

  /**
   * Return the heading of the robot in degrees
   * @return Current heading of the robot in degrees
   */
  public Measure<Angle> getAngle() {
    return m_navx.getInputs().yawAngle;
  }

  /**
   * Get rotate rate of robot
   * @return Current rotate rate of robot
   */
  public Measure<Velocity<Angle>> getRotateRate() {
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
