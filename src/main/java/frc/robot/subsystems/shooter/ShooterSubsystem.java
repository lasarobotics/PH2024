// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.List;
import java.util.Map.Entry;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.util.Precision;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.FeedbackSensor;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;
import org.lasarobotics.led.LEDStrip;
import org.lasarobotics.led.LEDStrip.Pattern;
import org.lasarobotics.led.LEDSubsystem;
import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private Spark topFlywheelMotor;
    private Spark bottomFlywheelMotor;
    private Spark angleMotor;
    private Spark indexerMotor;
    private LEDStrip ledStrip;

    public Hardware(Spark lSlaveMotor,
                    Spark rMasterMotor,
                    Spark angleMotor,
                    Spark indexerMotor,
                    LEDStrip ledStrip) {
      this.topFlywheelMotor = lSlaveMotor;
      this.bottomFlywheelMotor = rMasterMotor;
      this.angleMotor = angleMotor;
      this.indexerMotor = indexerMotor;
      this.ledStrip = ledStrip;
    }
  }

  /** Shooter state */
  public static class State {
    public final Measure<Velocity<Distance>> speed;
    public final Measure<Angle> angle;

    public static final State AMP_PREP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(56.0));
    public static final State AMP_SCORE_STATE = new State(Units.MetersPerSecond.of(+3.1), Units.Degrees.of(56.0));
    public static final State SPEAKER_PREP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(53.0));
    public static final State SPEAKER_SCORE_STATE = new State(Units.MetersPerSecond.of(+15.0), Units.Degrees.of(53.0));
    public static final State SOURCE_PREP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(55.0));
    public static final State SOURCE_INTAKE_STATE = new State(Units.MetersPerSecond.of(-10.0), Units.Degrees.of(55.0));
    public static final State PASSING_STATE = new State(Units.MetersPerSecond.of(+15.0), Units.Degrees.of(45.0));
    public static final State TEST_STOP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(30.0));

    public State(Measure<Velocity<Distance>> speed, Measure<Angle> angle) {
      this.speed = speed;
      this.angle = angle;
    }
  }

  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  public static final Measure<Velocity<Distance>> ZERO_FLYWHEEL_SPEED = Units.MetersPerSecond.of(0.0);
  private static final Measure<Current> FLYWHEEL_CURRENT_LIMIT = Units.Amps.of(80.0);
  private static final Measure<Current> ANGLE_MOTOR_CURRENT_LIMIT = Units.Amps.of(50.0);
  private static final Measure<Dimensionless> INDEXER_SPEED = Units.Percent.of(100.0);
  private static final Measure<Dimensionless> INDEXER_SLOW_SPEED = Units.Percent.of(4.0);
  private static final String MECHANISM_2D_LOG_ENTRY = "/Mechanism2d";
  private static final String SHOOTER_STATE_FLYWHEEL_SPEED = "/CurrentState/FlywheelSpeed";
  private static final String SHOOTER_STATE_ANGLE_DEGREES = "/CurrentState/Angle";
  private static final String SHOOTER_DESIRED_STATE_ANGLE = "/DesiredState/Angle";
  private static final String SHOOTER_DESIRED_STATE_SPEED = "/DesiredState/FlywheelSpeed";
  private static final String SHOOTER_TARGET_DISTANCE = "/TargetDistance";
  private static final String SHOOTER_NOTE_INSIDE_INDICATOR = "Note";

  private final Measure<Distance> MIN_SHOOTING_DISTANCE = Units.Meters.of(0.0);
  private final Measure<Distance> MAX_SHOOTING_DISTANCE;
  private final Measure<Velocity<Distance>> MAX_FLYWHEEL_SPEED;
  private final Measure<Velocity<Distance>> SPINUP_SPEED = Units.MetersPerSecond.of(10.0);

  private final Measure<Current> NOTE_SHOT_CURRENT_THRESHOLD = Units.Amps.of(10.0);
  private final Measure<Time> NOTE_SHOT_TIME_THRESHOLD = Units.Seconds.of(0.1);
  private final Measure<Time> READY_TIME_THRESHOLD = Units.Seconds.of(GlobalConstants.ROBOT_LOOP_PERIOD * 2);
  private final Debouncer NOTE_SHOT_DETECTOR = new Debouncer(NOTE_SHOT_TIME_THRESHOLD.in(Units.Seconds), DebounceType.kRising);
  private final Debouncer READY_DEBOUNCER = new Debouncer(READY_TIME_THRESHOLD.in(Units.Seconds), DebounceType.kBoth);

  private Spark m_topFlywheelMotor;
  private Spark m_bottomFlywheelMotor;
  private Spark m_angleMotor;
  private Spark m_indexerMotor;
  private LEDStrip m_ledStrip;

  private TrapezoidProfile.Constraints m_angleConstraint;
  private Supplier<Pose2d> m_poseSupplier;
  private Supplier<AprilTag> m_targetSupplier;

  private SparkPIDConfig m_flywheelConfig;
  private SparkPIDConfig m_angleConfig;

  private State m_desiredShooterState;
  private PolynomialSplineFunction m_shooterAngleCurve;
  private PolynomialSplineFunction m_shooterFlywheelCurve;

  private Mechanism2d m_mechanism2d;
  private MechanismLigament2d m_simShooterJoint;
  private TrapezoidProfile m_simShooterAngleMotionProfile;
  private TrapezoidProfile.State m_simShooterAngleState;
  private Measure<Distance> m_targetDistance;

  /**
   * Create an instance of ShooterSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param shooterHardware Hardware devices required by shooter
   * @param flywheelConfig Flywheel PID config
   * @param angleConfig Angle adjust PID config
   * @param angleFF Angle adjust feed forward gains
   * @param angleConstraint Angle adjust motion constraint
   * @param topFlywheelDiameter Top flywheel diameter
   * @param bottomFlywheelDiameter Bottom flywheel diameter
   * @param shooterMap Shooter lookup table
   * @param poseSupplier Robot pose supplier
   * @param targetSupplier Speaker target supplier
   */
  public ShooterSubsystem(Hardware shooterHardware, SparkPIDConfig flywheelConfig, SparkPIDConfig angleConfig,
                          TrapezoidProfile.Constraints angleConstraint, Measure<Distance> topFlywheelDiameter, Measure<Distance> bottomFlywheelDiameter,
                          List<Entry<Measure<Distance>, State>> shooterMap,
                          Supplier<Pose2d> poseSupplier, Supplier<AprilTag> targetSupplier) {
    setSubsystem(getClass().getSimpleName());
    MAX_FLYWHEEL_SPEED = Units.MetersPerSecond.of((shooterHardware.topFlywheelMotor.getKind().getMaxRPM() / 60) * (topFlywheelDiameter.in(Units.Meters) * Math.PI));
    this.m_topFlywheelMotor = shooterHardware.topFlywheelMotor;
    this.m_bottomFlywheelMotor = shooterHardware.bottomFlywheelMotor;
    this.m_angleMotor = shooterHardware.angleMotor;
    this.m_indexerMotor = shooterHardware.indexerMotor;
    this.m_ledStrip = shooterHardware.ledStrip;
    this.m_flywheelConfig = flywheelConfig;
    this.m_angleConfig = angleConfig;
    this.m_angleConstraint = angleConstraint;
    this.m_poseSupplier = poseSupplier;
    this.m_targetSupplier = targetSupplier;


    // Initialize PID
    m_topFlywheelMotor.initializeSparkPID(m_flywheelConfig, FeedbackSensor.NEO_ENCODER);
    m_bottomFlywheelMotor.initializeSparkPID(m_flywheelConfig, FeedbackSensor.NEO_ENCODER);
    m_angleMotor.initializeSparkPID(m_angleConfig, FeedbackSensor.THROUGH_BORE_ENCODER, true, true);

    // Set flywheel conversion factor
    var topFlywheelConversionFactor = topFlywheelDiameter.in(Units.Meters) * Math.PI;
    var bottomFlywheelConversionFactor = bottomFlywheelDiameter.in(Units.Meters) * Math.PI;
    m_topFlywheelMotor.setPositionConversionFactor(FeedbackSensor.NEO_ENCODER, topFlywheelConversionFactor);
    m_topFlywheelMotor.setVelocityConversionFactor(FeedbackSensor.NEO_ENCODER, topFlywheelConversionFactor / 60);
    m_bottomFlywheelMotor.setPositionConversionFactor(FeedbackSensor.NEO_ENCODER, bottomFlywheelConversionFactor);
    m_bottomFlywheelMotor.setVelocityConversionFactor(FeedbackSensor.NEO_ENCODER, bottomFlywheelConversionFactor / 60);

    // Set angle adjust conversion factor
    var angleConversionFactor = Math.PI * 2;
    m_angleMotor.setPositionConversionFactor(FeedbackSensor.THROUGH_BORE_ENCODER, angleConversionFactor);
    m_angleMotor.setVelocityConversionFactor(FeedbackSensor.THROUGH_BORE_ENCODER, angleConversionFactor / 60);

    // Set idle mode
    m_topFlywheelMotor.setIdleMode(IdleMode.kCoast);
    m_bottomFlywheelMotor.setIdleMode(IdleMode.kCoast);
    m_angleMotor.setIdleMode(IdleMode.kBrake);
    m_indexerMotor.setIdleMode(IdleMode.kBrake);

    // Set current limits
    m_topFlywheelMotor.setSmartCurrentLimit(FLYWHEEL_CURRENT_LIMIT);
    m_bottomFlywheelMotor.setSmartCurrentLimit(FLYWHEEL_CURRENT_LIMIT);
    m_angleMotor.setSmartCurrentLimit(ANGLE_MOTOR_CURRENT_LIMIT);

    // Disable indexer hard limits
    m_indexerMotor.disableForwardLimitSwitch();
    m_indexerMotor.disableReverseLimitSwitch();

    // Initialize shooter state
    m_desiredShooterState = getCurrentState();

    // Register LED strip with LED subsystem
    LEDSubsystem.getInstance().add(m_ledStrip);

    // Set LED strip to team color
    m_ledStrip.set(Pattern.TEAM_COLOR_SOLID);

    // Set maximum shooting distance
    MAX_SHOOTING_DISTANCE = shooterMap.get(shooterMap.size() - 1).getKey();

    // Initialize shooter curves
    initializeShooterCurves(shooterMap);

    // Set default command to track speaker angle
    setDefaultCommand(run(() -> {
      var state = getAutomaticState();
      state = new State(SPINUP_SPEED, state.angle);
      setState(state);
    }));

    // Initialize sim variables
    m_mechanism2d = new Mechanism2d(1.0, 1.0);
    m_simShooterJoint = m_mechanism2d.getRoot("shooter", 0.5, 0.33).append(new MechanismLigament2d("shooter", 0.4, 1.0));
    m_simShooterAngleMotionProfile = new TrapezoidProfile(m_angleConstraint);
    m_simShooterAngleState = new TrapezoidProfile.State(m_angleConfig.getLowerLimit(), 0.0);

    // Register LED strip with LED subsystem
    LEDSubsystem.getInstance().add(m_ledStrip);

    // Set LEDs to team color
    m_ledStrip.set(Pattern.TEAM_COLOR_BREATHE);
  }

  /**
   * Initialize hardware devices for shooter subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware shooterHardware = new Hardware(
      new Spark(Constants.ShooterHardware.TOP_FLYWHEEL_MOTOR_ID, MotorKind.NEO_VORTEX),
      new Spark(Constants.ShooterHardware.BOTTOM_FLYWHEEL_MOTOR_ID, MotorKind.NEO_VORTEX),
      new Spark(Constants.ShooterHardware.ANGLE_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.ShooterHardware.INDEXER_MOTOR_ID, MotorKind.NEO),
      new LEDStrip(LEDStrip.initializeHardware(Constants.ShooterHardware.LED_STRIP_ID))
    );

    return shooterHardware;
  }

  /**
   * Initialize spline functions for shooter
   * @param shooterMap List of distance and shooter state pairs
   */
  private void initializeShooterCurves(List<Entry<Measure<Distance>, State>> shooterMap) {
    double[] distances = new double[shooterMap.size()];
    double[] flywheelSpeeds = new double[shooterMap.size()];
    double[] angles = new double[shooterMap.size()];

    for (int i = 0; i < shooterMap.size(); i++) {
      distances[i] = shooterMap.get(i).getKey().in(Units.Meters);
      flywheelSpeeds[i] = shooterMap.get(i).getValue().speed.in(Units.MetersPerSecond);
      angles[i] = shooterMap.get(i).getValue().angle.in(Units.Radians);
    }

    m_shooterFlywheelCurve = SPLINE_INTERPOLATOR.interpolate(distances, flywheelSpeeds);
    m_shooterAngleCurve = SPLINE_INTERPOLATOR.interpolate(distances, angles);
  }

  /**
   * Set shooter to desired state
   * @param state Desired shooter state
   */
  private void setState(State state) {
    // Normalize state to valid range
    m_desiredShooterState = normalizeState(state);

    if (state.speed.isNear(ZERO_FLYWHEEL_SPEED, 0.01)) {
      m_topFlywheelMotor.stopMotor();
      m_bottomFlywheelMotor.stopMotor();
    }
    else {
      m_topFlywheelMotor.set(m_desiredShooterState.speed.in(Units.MetersPerSecond), ControlType.kVelocity);
      m_bottomFlywheelMotor.set(m_desiredShooterState.speed.in(Units.MetersPerSecond), ControlType.kVelocity);
    }
    m_angleMotor.set(m_desiredShooterState.angle.in(Units.Radians), ControlType.kPosition);
  }

  /**
   * Normalize shooter state to be within valid values
   * @param state Desired state
   * @return Valid shooter state
   */
  private State normalizeState(State state) {
    Measure<Velocity<Distance>> clampedSpeed = Units.MetersPerSecond.of(MathUtil.clamp(
      state.speed.in(Units.MetersPerSecond),
      -MAX_FLYWHEEL_SPEED.in(Units.MetersPerSecond),
      +MAX_FLYWHEEL_SPEED.in(Units.MetersPerSecond)
    ));
    Measure<Angle> clampedAngle = Units.Radians.of(MathUtil.clamp(
      state.angle.in(Units.Radians),
      m_angleConfig.getLowerLimit(),
      m_angleConfig.getUpperLimit()
    ));
    return new State(clampedSpeed, clampedAngle);
  }

  /**
   * Reset shooter state
   */
  private void resetState() {
    setState(new State(SPINUP_SPEED, m_desiredShooterState.angle));
  }

  /**
   * Get current shooter state
   * @return Current shooter state
   */
  private State getCurrentState() {
    return new State(
      Units.MetersPerSecond.of(m_topFlywheelMotor.getInputs().encoderVelocity),
      Units.Radians.of(m_angleMotor.getInputs().absoluteEncoderPosition)
    );
  }

  /**
   * Get shooter state based on distance to target
   * @return Shooter state for current target distance
   */
  private State getAutomaticState() {
    var targetDistance = getTargetDistance();
    var flywheelSpeed = m_shooterFlywheelCurve.value(targetDistance.in(Units.Meters));
    var angle = m_shooterAngleCurve.value(targetDistance.in(Units.Meters));

    return new State(Units.MetersPerSecond.of(flywheelSpeed), Units.Radians.of(angle));
  }

  /**
   * Get distance to target, clamped to maximum shooting distance
   * @return Distance to target
   */
  private Measure<Distance> getTargetDistance() {
    return m_targetDistance;
  }

  /**
   * Check if shooter has reached desired state and is ready
   * @return True if ready
   */
  private boolean isReady() {
    return READY_DEBOUNCER.calculate(
      Precision.equals(
        m_angleMotor.getInputs().absoluteEncoderPosition,
        m_desiredShooterState.angle.in(Units.Radians),
        m_angleConfig.getTolerance()
      ) &&
      Precision.equals(
        m_topFlywheelMotor.getInputs().encoderVelocity,
        m_desiredShooterState.speed.in(Units.MetersPerSecond),
        m_flywheelConfig.getTolerance()
      ) &&
      Precision.equals(
        m_bottomFlywheelMotor.getInputs().encoderVelocity,
        m_desiredShooterState.speed.in(Units.MetersPerSecond),
        m_flywheelConfig.getTolerance()
      ));
  }

  /**
   * Feed game piece to flywheels
   * @param slow True to run slowly
   */
  private void feedStart(boolean slow) {
    m_indexerMotor.set(slow ? +INDEXER_SLOW_SPEED.in(Units.Percent) : +INDEXER_SPEED.in(Units.Percent));
  }

  /**
   * Reverse indexer
   * @param slow True to run slowly
   */
  private void feedReverse(boolean slow) {
    m_indexerMotor.set(slow ? -INDEXER_SLOW_SPEED.in(Units.Percent) : -INDEXER_SPEED.in(Units.Percent));
  }

  /**
   * Stop feeding
   */
  private void feedStop() {
    m_indexerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_topFlywheelMotor.periodic();
    m_bottomFlywheelMotor.periodic();
    m_angleMotor.periodic();
    m_indexerMotor.periodic();

    // Put note indicator on SmartDashboard
    SmartDashboard.putBoolean(SHOOTER_NOTE_INSIDE_INDICATOR, isObjectPresent());

    // Update target distance
    m_targetDistance = Units.Meters.of(MathUtil.clamp(
      m_poseSupplier.get().getTranslation().getDistance(m_targetSupplier.get().pose.toPose2d().getTranslation()),
      MIN_SHOOTING_DISTANCE.in(Units.Meters),
      MAX_SHOOTING_DISTANCE.in(Units.Meters)
    ));

    // Log outputs
    var currentState = getCurrentState();
    Logger.recordOutput(getName() + MECHANISM_2D_LOG_ENTRY, m_mechanism2d);
    Logger.recordOutput(getName() + SHOOTER_STATE_FLYWHEEL_SPEED, currentState.speed.in(Units.MetersPerSecond));
    Logger.recordOutput(getName() + SHOOTER_STATE_ANGLE_DEGREES, currentState.angle.in(Units.Degrees));
    Logger.recordOutput(getName() + SHOOTER_DESIRED_STATE_ANGLE, m_desiredShooterState.angle.in(Units.Degrees));
    Logger.recordOutput(getName() + SHOOTER_DESIRED_STATE_SPEED, m_desiredShooterState.speed.in(Units.MetersPerSecond));
    Logger.recordOutput(getName() + SHOOTER_TARGET_DISTANCE, getTargetDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run in simulation
    m_topFlywheelMotor.getInputs().encoderVelocity = m_desiredShooterState.speed.in(Units.MetersPerSecond);
    m_bottomFlywheelMotor.getInputs().encoderVelocity = m_desiredShooterState.speed.in(Units.MetersPerSecond);

    m_angleMotor.getInputs().absoluteEncoderPosition = m_simShooterAngleState.position;
    m_angleMotor.getInputs().absoluteEncoderVelocity = m_simShooterAngleState.velocity;

    m_simShooterAngleState = m_simShooterAngleMotionProfile.calculate(
      GlobalConstants.ROBOT_LOOP_PERIOD,
      m_simShooterAngleState,
      new TrapezoidProfile.State(m_desiredShooterState.angle.in(Units.Radians), 0.0)
    );

    m_simShooterJoint.setAngle(Rotation2d.fromRadians(Math.PI - m_simShooterAngleState.position));
  }

  /**
   * Intake a game piece from the ground intake to be later fed to the shooter
   * @return Command to intake to the shooter
   */
  public Command intakeCommand() {
    return runEnd(
      () -> getDefaultCommand().execute(),
      () -> {
        m_indexerMotor.disableForwardLimitSwitch();
        feedStop();
      }
    ).beforeStarting(() -> {
        m_indexerMotor.enableForwardLimitSwitch();
        feedStart(true);
    }).until(() -> isObjectPresent());
  }

  /**
   * Intake a game piece from the source via the shooter
   * @return Command that intakes via the shooter
   */
  public Command sourceIntakeCommand() {
    return startEnd(
      () -> {
        m_indexerMotor.enableReverseLimitSwitch();
        feedReverse(true);
        setState(State.SOURCE_INTAKE_STATE);
      },
      () -> {
        m_indexerMotor.disableReverseLimitSwitch();
        feedStop();
        resetState();
      }
    ).until(() -> isObjectPresent());
  }

  /**
   * Intake a game piece from the ground intake to be later fed to the shooter with no limit switches
   * @return Command to feed through the shooter
   */
  public Command feedCommand() {
    return startEnd(() -> feedStart(false), () -> feedStop());
  }

  public Command feedThroughCommand(BooleanSupplier isAimed) {
    return startEnd(() -> {
      feedStart(false);
      setState(new State(Units.MetersPerSecond.of(2.0), m_desiredShooterState.angle));
    },
    () -> {
      feedStop();
      resetState();
    });
  }

  /**
   * Reverse note from shooter into intake
   * @return Command to outtake note in shooter
   */
  public Command outtakeCommand() {
    return startEnd(
      () -> feedReverse(false),
      () -> feedStop()
    );
  }

  /**
   * Shoot by manually setting shooter state
   * @param stateSupplier Desired shooter state supplier
   * @return Command to control shooter manually
   */
  public Command shootManualCommand(Supplier<State> stateSupplier) {
    return runEnd(
      () -> {
        if (isReady()) feedStart(false);
        else feedStop();
      },
      () -> {
        feedStop();
        resetState();
      }
    ).beforeStarting(() -> setState(stateSupplier.get()), this);
  }

  /**
   * Shoot by manually setting shooter state
   * @param state Desired shooter state
   * @return Command to control shooter manually
   */
  public Command shootManualCommand(State state) {
    return shootManualCommand(() -> state);
  }

  /**
   * Shoot automatically based on current location
   * @param isAimed Is robot aimed at target
   * @param override Shoot even if target tag is not visible and not in range
   * @return Command to automatically shoot note
   */
  public Command shootCommand(BooleanSupplier isAimed, BooleanSupplier override) {
    return runEnd(
      () -> {
        setState(getAutomaticState());
        if ((RobotBase.isSimulation() | isReady() && isAimed.getAsBoolean()) || override.getAsBoolean())
          feedStart(false);
        else feedStop();
      },
      () -> {
        feedStop();
        resetState();
      }
    );
  }

 /**
   * Shoot automatically based on current location, checking if target tag is visible and robot is in range
   * @param isAimed Is robot aimed at target
   * @return Command to automatically shoot note
   */
  public Command shootCommand(BooleanSupplier isAimed) {
    return shootCommand(isAimed, () -> false);
  }

  /**
   * Shoot note into speaker from subwoofer
   * @return Command to shoot note when parked against the subwoofer
   */
  public Command shootSpeakerCommand() {
    return shootManualCommand(State.SPEAKER_SCORE_STATE);
  }

  /**
   * Pass note from opponent side to your side
   * @return Command to pass note
   */
  public Command passCommand() {
    return shootManualCommand(State.PASSING_STATE);
  }

  /**
   * Move shooter up and down
   * @return Command to move the shooter between upper and lower limits
   */
  public Command shootPartyMode() {
    final State TOP = new State(ZERO_FLYWHEEL_SPEED, Units.Radians.of(m_angleConfig.getUpperLimit()));
    final State BOTTOM = new State(ZERO_FLYWHEEL_SPEED, Units.Radians.of(m_angleConfig.getLowerLimit()));

    return Commands.sequence(
      run(() -> setState(TOP)).until(() -> isReady()),
      run(() -> setState(BOTTOM)).until(() -> isReady())
    ).repeatedly();
  }

  /**
   * Move shooter to amp position
   * @return Command that prepares shooter for scoring in the amp
   */
  public Command prepareForAmpCommand() {
    return startEnd(
      () -> setState(State.AMP_PREP_STATE),
      () -> resetState()
    ).until(() -> isReady());
  }

  /**
   * Score note in amp
   * @return Command that shoots note into amp
   */
  public Command scoreAmpCommand() {
    return shootManualCommand(State.AMP_SCORE_STATE);
  }

  /**
   * Spin up flywheel
   * @return Command that spins up flywheel
   */
  public Command spinupCommand() {
    return run(() -> setState(new State(SPINUP_SPEED, m_desiredShooterState.angle)));
  }

  /**
   * Whether a game piece is in the indexer
   * @return The value of the indexer motor's forward limit switch
   */
  public boolean isObjectPresent() {
    return m_indexerMotor.getInputs().forwardLimitSwitch;
  }

  /**
   * Whether a note has been shot
   * @return If the current of the top flywheel motor is greater than the threshold for a specified time
   */
  public boolean hasBeenShot() {
    return NOTE_SHOT_DETECTOR.calculate(m_topFlywheelMotor.getOutputCurrent().compareTo(NOTE_SHOT_CURRENT_THRESHOLD) > 0);
  }

  @Override
  public void close() {
    m_topFlywheelMotor.close();
    m_bottomFlywheelMotor.close();
    m_angleMotor.close();
    m_indexerMotor.close();
  }
}
