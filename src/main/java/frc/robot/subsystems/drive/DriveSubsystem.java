// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.concurrent.ThreadLocalRandom;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.lasarobotics.drive.AdvancedSwerveKinematics;
import org.lasarobotics.drive.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.MAXSwerveModule;
import org.lasarobotics.drive.RotatePIDController;
import org.lasarobotics.drive.ThrottleMap;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
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
import edu.wpi.first.math.filter.MedianFilter;
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
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
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
  public static final Measure<Distance> DRIVE_WHEELBASE = Units.Meters.of(0.62);
  public static final Measure<Distance> DRIVE_TRACK_WIDTH = Units.Meters.of(0.62);
  public static final Measure<Time> AUTO_LOCK_TIME = Units.Seconds.of(3.0);
  public static final Measure<Time> MAX_SLIPPING_TIME = Units.Seconds.of(0.9);
  public static final Measure<Current> DRIVE_CURRENT_LIMIT = Units.Amps.of(45.0);
  public static final Measure<Velocity<Angle>> NAVX2_YAW_DRIFT_RATE = Units.DegreesPerSecond.of(0.5 / 60);
  public static final Measure<Velocity<Angle>> DRIVE_ROTATE_VELOCITY = Units.RadiansPerSecond.of(12 * Math.PI);
  public static final Measure<Velocity<Velocity<Angle>>> DRIVE_ROTATE_ACCELERATION = Units.RadiansPerSecond.of(4 * Math.PI).per(Units.Second);
  public final Measure<Velocity<Distance>> DRIVE_MAX_LINEAR_SPEED;
  public final Measure<Velocity<Velocity<Distance>>> DRIVE_AUTO_ACCELERATION;

  // Other settings
  private static final int INERTIAL_VELOCITY_FILTER_TAPS = 100;
  private static final double TOLERANCE = 1.0;
  private static final double TIP_THRESHOLD = 35.0;
  private static final double BALANCED_THRESHOLD = 10.0;
  private static final double AIM_VELOCITY_COMPENSATION_FUDGE_FACTOR = 0.3;
  private static final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.03, 0.03, Math.toRadians(1));
  private static final Matrix<N3, N1> VISION_STDDEV = VecBuilder.fill(0.5, 0.5, Math.toRadians(40));
  private static final TrapezoidProfile.Constraints AIM_PID_CONSTRAINT = new TrapezoidProfile.Constraints(2160.0, 2160.0);

  // Log
  private static final String POSE_LOG_ENTRY = "/Pose";
  private static final String ACTUAL_SWERVE_STATE_LOG_ENTRY = "/ActualSwerveState";
  private static final String DESIRED_SWERVE_STATE_LOG_ENTRY = "/DesiredSwerveState";


  private ThrottleMap m_throttleMap;
  private RotatePIDController m_rotatePIDController;
  private ProfiledPIDController m_autoAimPIDControllerFront;
  private ProfiledPIDController m_autoAimPIDControllerBack;
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

  private ControlCentricity m_controlCentricity;
  private ChassisSpeeds m_desiredChassisSpeeds;
  private boolean m_isTractionControlEnabled = true;
  private Pose2d m_previousPose;
  private Rotation2d m_currentHeading;
  private PurplePathClient m_purplePathClient;
  private Field2d m_field;
  private MedianFilter m_xVelocityFilter;
  private MedianFilter m_yVelocityFilter;

  public final Command ANTI_TIP_COMMAND = new FunctionalCommand(
    () -> m_ledStrip.set(Pattern.RED_STROBE),
    () -> antiTip(),
    (interrupted) -> {
      m_ledStrip.set(Pattern.GREEN_SOLID);
      resetRotatePID();
      stop();
      lock();
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
    this.m_ledStrip = drivetrainHardware.ledStrip;
    this.m_controlCentricity = controlCentricity;
    this.m_throttleMap = new ThrottleMap(throttleInputCurve, DRIVE_MAX_LINEAR_SPEED, deadband);
    this.m_rotatePIDController = new RotatePIDController(turnInputCurve, pidf, turnScalar, deadband, lookAhead);
    this.m_pathFollowerConfig = new HolonomicPathFollowerConfig(
      new com.pathplanner.lib.util.PIDConstants(5.0, 0.0, 0.0),
      new com.pathplanner.lib.util.PIDConstants(5.0, 0.0, 0.1),
      DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond),
      m_lFrontModule.getModuleCoordinate().getNorm(),
      new ReplanningConfig(),
      GlobalConstants.ROBOT_LOOP_PERIOD
    );
    this.m_xVelocityFilter = new MedianFilter(INERTIAL_VELOCITY_FILTER_TAPS);
    this.m_yVelocityFilter = new MedianFilter(INERTIAL_VELOCITY_FILTER_TAPS);

    // Calibrate and reset navX
    while (m_navx.isCalibrating()) stop();
    m_navx.reset();

    // Setup turn PID
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
    m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      getRotation2d(),
      getModulePositions(),
      new Pose2d(),
      ODOMETRY_STDDEV,
      VISION_STDDEV
    );

    // Initialise chassis speeds
    m_desiredChassisSpeeds = new ChassisSpeeds();

    // Setup anti-tip command
    new Trigger(this::isTipping).whileTrue(ANTI_TIP_COMMAND);

    // Register LED strip with LED subsystem
    LEDSubsystem.getInstance().add(m_ledStrip);

    // Set LED strip to team color
    m_ledStrip.set(Pattern.TEAM_COLOR_SOLID);

    // Setup auto-aim PID controller
    m_autoAimPIDControllerFront = new ProfiledPIDController(pidf.kP, 0.0, pidf.kD, AIM_PID_CONSTRAINT, pidf.period);
    m_autoAimPIDControllerFront.enableContinuousInput(-180.0, +180.0);
    m_autoAimPIDControllerFront.setTolerance(TOLERANCE);
    m_autoAimPIDControllerBack = new ProfiledPIDController(pidf.kP, 0.0, pidf.kD, AIM_PID_CONSTRAINT, pidf.period);
    m_autoAimPIDControllerBack.enableContinuousInput(-180.0, +180.0);
    m_autoAimPIDControllerBack.setTolerance(TOLERANCE);

    // Initialise other variables
    m_previousPose = new Pose2d();
    m_currentHeading = new Rotation2d();

    // Initalise PurplePathClient
    m_purplePathClient = new PurplePathClient(this);

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
      MAXSwerveModule.ModuleLocation.LeftFront,
      Constants.Drive.GEAR_RATIO,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      AUTO_LOCK_TIME,
      MAX_SLIPPING_TIME,
      DRIVE_CURRENT_LIMIT,
      Constants.Drive.DRIVE_SLIP_RATIO
    );

    MAXSwerveModule rFrontModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.RIGHT_FRONT_DRIVE_MOTOR_ID,
        Constants.DriveHardware.RIGHT_FRONT_ROTATE_MOTOR_ID,
        MotorKind.NEO_VORTEX
      ),
      MAXSwerveModule.ModuleLocation.RightFront,
      Constants.Drive.GEAR_RATIO,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      AUTO_LOCK_TIME,
      MAX_SLIPPING_TIME,
      DRIVE_CURRENT_LIMIT,
      Constants.Drive.DRIVE_SLIP_RATIO
    );

    MAXSwerveModule lRearModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.LEFT_REAR_DRIVE_MOTOR_ID,
        Constants.DriveHardware.LEFT_REAR_ROTATE_MOTOR_ID,
        MotorKind.NEO_VORTEX
      ),
      MAXSwerveModule.ModuleLocation.LeftRear,
      Constants.Drive.GEAR_RATIO,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      AUTO_LOCK_TIME,
      MAX_SLIPPING_TIME,
      DRIVE_CURRENT_LIMIT,
      Constants.Drive.DRIVE_SLIP_RATIO
    );

    MAXSwerveModule rRearModule = new MAXSwerveModule(
      MAXSwerveModule.initializeHardware(
        Constants.DriveHardware.RIGHT_REAR_DRIVE_MOTOR_ID,
        Constants.DriveHardware.RIGHT_REAR_ROTATE_MOTOR_ID,
        MotorKind.NEO_VORTEX
      ),
      MAXSwerveModule.ModuleLocation.RightRear,
      Constants.Drive.GEAR_RATIO,
      DRIVE_WHEELBASE,
      DRIVE_TRACK_WIDTH,
      AUTO_LOCK_TIME,
      MAX_SLIPPING_TIME,
      DRIVE_CURRENT_LIMIT,
      Constants.Drive.DRIVE_SLIP_RATIO
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
   * @param inertialVelocity Current inertial velocity
   * @param rotateRate Desired robot rotate rate
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
   * @param rotateRate Current robot rotate rate
   */
  private void drive(Measure<Velocity<Distance>> xRequest,
                     Measure<Velocity<Distance>> yRequest,
                     Measure<Velocity<Angle>> rotateRequest,
                     Measure<Velocity<Distance>> inertialVelocity) {
    // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
      new ChassisSpeeds(xRequest, yRequest, rotateRequest)
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
    setSwerveModules(moduleStates, inertialVelocity, Units.RadiansPerSecond.of(m_desiredChassisSpeeds.omegaRadiansPerSecond));
  }

  /**
   * Drive robot without traction control
   * @param xRequest Desired X (forward) velocity
   * @param yRequest Desired Y (sideways) velocity
   * @param rotateRequest Desired rotate rate
   */
  private void drive(Measure<Velocity<Distance>> xRequest,
                     Measure<Velocity<Distance>> yRequest,
                     Measure<Velocity<Angle>> rotateRequest) {
    // Get requested chassis speeds, correcting for second order kinematics
    m_desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
      new ChassisSpeeds(xRequest, yRequest, rotateRequest)
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
    for (var visionEstimatedRobotPose : visionEstimatedRobotPoses) {
      // if (visionEstimatedRobotPose.estimatedPose.toPose2d().getTranslation().getDistance(m_previousPose.getTranslation()) > 1.0) continue;
      m_poseEstimator.addVisionMeasurement(visionEstimatedRobotPose.estimatedPose.toPose2d(), visionEstimatedRobotPose.timestampSeconds);
    }
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

  /**
   * Start calling this repeatedly when robot is in danger of tipping over
   */
  private void antiTip() {
    // Calculate direction of tip
    double direction = Math.atan2(getRoll().in(Units.Degrees), getPitch().in(Units.Degrees));

    // Drive to counter tipping motion
    drive(
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
  private void aimAtPoint(double xRequest, double yRequest, double rotateRequest, Translation2d point, boolean reversed, boolean velocityCorrection) {
    // Calculate desired robot velocity
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);
    double velocityOutput = m_throttleMap.throttleLookup(moveRequest);

    // Drive normally and return if invalid point
    if (point == null) {
      double rotateOutput = -m_rotatePIDController.calculate(getAngle(), getRotateRate(), rotateRequest);
      drive(
        Units.MetersPerSecond.of(-velocityOutput * Math.cos(moveDirection)),
        Units.MetersPerSecond.of(-velocityOutput * Math.sin(moveDirection)),
        Units.DegreesPerSecond.of(rotateOutput),
        getInertialVelocity()
      );
      return;
    }

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
    Vector2D perpendicularRobotVector = robotVector.subtract(parallelRobotVector).scalarMultiply(velocityCorrection ? AIM_VELOCITY_COMPENSATION_FUDGE_FACTOR : 0.0);
    // Adjust aim point using calculated vector
    Translation2d adjustedPoint = point.minus(new Translation2d(perpendicularRobotVector.getX(), perpendicularRobotVector.getY()));
    // Calculate new angle using adjusted point
    Rotation2d adjustedAngle = new Rotation2d(adjustedPoint.getX() - currentPose.getX(), adjustedPoint.getY() - currentPose.getY());
    // Calculate necessary rotate rate
    double rotateOutput = reversed
      ? m_autoAimPIDControllerBack.calculate(currentPose.getRotation().plus(GlobalConstants.ROTATION_PI).getDegrees(), adjustedAngle.getDegrees())
      : m_autoAimPIDControllerFront.calculate(currentPose.getRotation().getDegrees(), adjustedAngle.getDegrees());

    // Log aim point
    Logger.recordOutput(getName() + "/AimPoint", new Pose2d(aimPoint, new Rotation2d()));

    // Drive robot accordingly
    drive(
      Units.MetersPerSecond.of(-velocityOutput * Math.cos(moveDirection)),
      Units.MetersPerSecond.of(-velocityOutput * Math.sin(moveDirection)),
      Units.DegreesPerSecond.of(rotateOutput),
      getInertialVelocity()
    );
  }

  /**
   * Aim robot by given angle
   * @param angle Desired angle in degrees
   */
  private void aimAtAngle(double angle) {
    double rotateOutput = m_rotatePIDController.calculate(getAngle().in(Units.Degrees), getAngle().in(Units.Degrees) + angle);

    drive(
      Units.MetersPerSecond.of(0),
      Units.MetersPerSecond.of(0),
      Units.DegreesPerSecond.of(rotateOutput),
      getInertialVelocity()
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
    double velocityOutput = m_throttleMap.throttleLookup(moveRequest);
    double rotateOutput = -m_rotatePIDController.calculate(getAngle(), getRotateRate(), rotateRequest);

    // Update auto-aim controllers
    m_autoAimPIDControllerFront.calculate(
      getPose().getRotation().getDegrees(),
      getPose().getRotation().getDegrees()
    );
    m_autoAimPIDControllerBack.calculate(
      getPose().getRotation().plus(GlobalConstants.ROTATION_PI).getDegrees(),
      getPose().getRotation().plus(GlobalConstants.ROTATION_PI).getDegrees()
    );

    // Drive robot
    drive(
      Units.MetersPerSecond.of(-velocityOutput * Math.cos(moveDirection)),
      Units.MetersPerSecond.of(-velocityOutput * Math.sin(moveDirection)),
      Units.DegreesPerSecond.of(rotateOutput),
      getInertialVelocity()
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
    m_poseEstimator.resetPosition(
      getRotation2d(),
      getModulePositions(),
      pose
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_navx.periodic();
    m_lFrontModule.periodic();
    m_rFrontModule.periodic();
    m_lRearModule.periodic();
    m_rRearModule.periodic();

    // Filter inertial velocity
    m_navx.getInputs().xVelocity = Units.MetersPerSecond.of(
      m_xVelocityFilter.calculate(m_navx.getInputs().xVelocity.in(Units.MetersPerSecond))
    );
    m_navx.getInputs().yVelocity = Units.MetersPerSecond.of(
      m_yVelocityFilter.calculate(m_navx.getInputs().yVelocity.in(Units.MetersPerSecond))
    );

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

    double randomNoise = ThreadLocalRandom.current().nextDouble(0.8, 1.0);
    m_navx.getInputs().xVelocity = Units.MetersPerSecond.of(m_desiredChassisSpeeds.vxMetersPerSecond * randomNoise);
    m_navx.getInputs().yVelocity = Units.MetersPerSecond.of(m_desiredChassisSpeeds.vyMetersPerSecond * randomNoise);
    m_navx.getInputs().yawRate = Units.RadiansPerSecond.of(m_desiredChassisSpeeds.omegaRadiansPerSecond * randomNoise);

    int yawDriftDirection = ThreadLocalRandom.current().nextDouble(1.0) < 0.5 ? -1 : +1;
    double angle = m_navx.getSimAngle() - Math.toDegrees(m_desiredChassisSpeeds.omegaRadiansPerSecond * randomNoise) * GlobalConstants.ROBOT_LOOP_PERIOD
                   + (NAVX2_YAW_DRIFT_RATE.in(Units.DegreesPerSecond) * GlobalConstants.ROBOT_LOOP_PERIOD * yawDriftDirection);
    m_navx.setSimAngle(angle);

    updatePose();
    smartDashboard();
    logOutputs();
  }

  /**
   * Configure PathPlanner auto builder
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
    return runEnd(() ->
      aimAtPoint(
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

  public void resetPosePathPlanner(Pose2d pose) {
    return;
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
      defer(() ->
        m_purplePathClient.getTrajectoryCommand(goal, parallelCommand)
        .finallyDo(() -> resetRotatePID())
      ),
      runOnce(() -> stop()),
      Commands.parallel(driveCommand(() -> 0.0, () -> 0.0, () -> 0.0), endCommand)
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
      DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond),
      DRIVE_AUTO_ACCELERATION.in(Units.MetersPerSecondPerSecond),
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

  public boolean isAimed() {
    return m_autoAimPIDControllerFront.atGoal() || m_autoAimPIDControllerBack.atGoal();
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
   * @return Current rotate rate of robot (degrees/s)
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
