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
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.FeedbackSensor;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private Spark topFlywheelMotor;
    private Spark bottomFlywheelMotor;
    private Spark angleMotor;
    private Spark indexerMotor;

    public Hardware(Spark lSlaveMotor,
                    Spark rMasterMotor,
                    Spark angleMotor,
                    Spark indexerMotor) {
      this.topFlywheelMotor = lSlaveMotor;
      this.bottomFlywheelMotor = rMasterMotor;
      this.angleMotor = angleMotor;
      this.indexerMotor = indexerMotor;
    }
  }

  /** Shooter state */
  public static class State {
    public final Measure<Velocity<Distance>> speed;
    public final Measure<Angle> angle;

    public static final State AMP_PREP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(90.0));
    public static final State AMP_SCORE_STATE = new State(Units.MetersPerSecond.of(0.0), Units.Degrees.of(90.0));
    public static final State SPEAKER_PREP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(90.0));
    public static final State SPEAKER_SCORE_STATE = new State(Units.MetersPerSecond.of(0.0), Units.Degrees.of(90.0));
    public static final State SOURCE_PREP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(90.0));

    public State(Measure<Velocity<Distance>> speed, Measure<Angle> angle) {
      this.speed = speed;
      this.angle = angle;
    }
  }

  public static final Measure<Angle> SHOOTER_ANGLE_OFFSET = Units.Degrees.of(20.0);
  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  private static final Measure<Velocity<Distance>> ZERO_FLYWHEEL_SPEED = Units.MetersPerSecond.of(0.0);
  private static final Measure<Current> FLYWHEEL_CURRENT_LIMIT = Units.Amps.of(40.0);
  private static final Measure<Current> ANGLE_MOTOR_CURRENT_LIMIT = Units.Amps.of(20.0);
  private static final Measure<Dimensionless> INDEXER_SPEED = Units.Percent.of(25.0);
  private static final String MECHANISM_2D_LOG_ENTRY = "/Mechanism2d";
  private static final String SHOOTER_STATE_FLYWHEEL_SPEED = "/CurrentState/FlywheelSpeed";
  private static final String SHOOTER_STATE_ANGLE_DEGREES = "/CurrentState/Angle";

  private final Measure<Distance> MIN_SHOOTING_DISTANCE = Units.Meters.of(0.0);
  private final Measure<Distance> MAX_SHOOTING_DISTANCE;
  private final Measure<Velocity<Distance>> MAX_FLYWHEEL_SPEED;

  private Spark m_topFlywheelMotor;
  private Spark m_bottomFlywheelMotor;
  private Spark m_angleMotor;
  private Spark m_indexerMotor;

  private ElevatorFeedforward m_angleFF;
  private TrapezoidProfile.Constraints m_angleConstraint;
  private Supplier<Pose2d> m_poseSupplier;
  private Supplier<Pair<Integer,Translation2d>> m_targetSupplier;

  private SparkPIDConfig m_flywheelConfig;
  private SparkPIDConfig m_angleConfig;

  private State m_desiredShooterState;
  private PolynomialSplineFunction m_shooterAngleCurve;
  private PolynomialSplineFunction m_shooterFlywheelCurve;

  private Mechanism2d m_mechanism2d;
  private MechanismLigament2d m_simShooterJoint;
  private TrapezoidProfile m_simShooterAngleMotionProfile;
  private TrapezoidProfile.State m_simShooterAngleState;

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
   * @param flywheelDiameter Flywheel diameter
   * @param shooterMap Shooter lookup table
   * @param poseSupplier Robot pose supplier
   * @param targetSupplier Speaker target supplier
   */
  public ShooterSubsystem(Hardware shooterHardware, SparkPIDConfig flywheelConfig, SparkPIDConfig angleConfig,
                          FFConstants angleFF, TrapezoidProfile.Constraints angleConstraint, Measure<Distance> flywheelDiameter,
                          List<Entry<Measure<Distance>, State>> shooterMap,
                          Supplier<Pose2d> poseSupplier, Supplier<Pair<Integer,Translation2d>> targetSupplier) {
    setSubsystem(getClass().getSimpleName());
    MAX_FLYWHEEL_SPEED = Units.MetersPerSecond.of((shooterHardware.topFlywheelMotor.getKind().getMaxRPM() / 60) * (flywheelDiameter.in(Units.Meters) * Math.PI));
    this.m_topFlywheelMotor = shooterHardware.topFlywheelMotor;
    this.m_bottomFlywheelMotor = shooterHardware.bottomFlywheelMotor;
    this.m_angleMotor = shooterHardware.angleMotor;
    this.m_indexerMotor = shooterHardware.indexerMotor;
    this.m_flywheelConfig = flywheelConfig;
    this.m_angleConfig = angleConfig;
    this.m_angleFF = new ElevatorFeedforward(angleFF.kS, angleFF.kG, angleFF.kV, angleFF.kA);
    this.m_angleConstraint = angleConstraint;
    this.m_poseSupplier = poseSupplier;
    this.m_targetSupplier = targetSupplier;

    // Slave bottom flywheel motor to top
    m_bottomFlywheelMotor.follow(m_topFlywheelMotor);

    // Initialize PID
    m_topFlywheelMotor.initializeSparkPID(m_flywheelConfig, FeedbackSensor.NEO_ENCODER);
    m_angleMotor.initializeSparkPID(m_angleConfig, FeedbackSensor.THROUGH_BORE_ENCODER, true, true);

    // Set flywheel conversion factor
    var flywheelConversionFactor = flywheelDiameter.in(Units.Meters) * Math.PI;
    m_topFlywheelMotor.setPositionConversionFactor(FeedbackSensor.NEO_ENCODER, flywheelConversionFactor);
    m_topFlywheelMotor.setVelocityConversionFactor(FeedbackSensor.NEO_ENCODER, flywheelConversionFactor / 60);

    // Set angle adjust conversion factor
    var angleConversionFactor = Math.PI * 2;
    m_angleMotor.setPositionConversionFactor(FeedbackSensor.NEO_ENCODER, angleConversionFactor);
    m_angleMotor.setVelocityConversionFactor(FeedbackSensor.NEO_ENCODER, angleConversionFactor / 60);

    // Set current limits
    m_topFlywheelMotor.setSmartCurrentLimit((int)FLYWHEEL_CURRENT_LIMIT.in(Units.Amps));
    m_bottomFlywheelMotor.setSmartCurrentLimit((int)FLYWHEEL_CURRENT_LIMIT.in(Units.Amps));
    m_angleMotor.setSmartCurrentLimit((int)ANGLE_MOTOR_CURRENT_LIMIT.in(Units.Amps));

    // Initialize shooter state
    m_desiredShooterState = getCurrentState();

    // Set maximum shooting distance
    MAX_SHOOTING_DISTANCE = shooterMap.get(shooterMap.size() - 1).getKey();

    // Initialize shooter curves
    initializeShooterCurves(shooterMap);

    // Initialize sim variables
    m_mechanism2d = new Mechanism2d(1.0, 1.0);
    m_simShooterJoint = m_mechanism2d.getRoot("shooter", 0.5, 0.33).append(new MechanismLigament2d("shooter", 0.4, 1.0));
    m_simShooterAngleMotionProfile = new TrapezoidProfile(m_angleConstraint);
    m_simShooterAngleState = new TrapezoidProfile.State(SHOOTER_ANGLE_OFFSET.in(Units.Radians), 0.0);
  }

  /**
   * Initialize hardware devices for shooter subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware shooterHardware = new Hardware(
      new Spark(Constants.ShooterHardware.TOP_FLYWHEEL_MOTOR_ID, MotorKind.NEO_VORTEX),
      new Spark(Constants.ShooterHardware.BOTTOM_FLYWHEEL_MOTOR_ID, MotorKind.NEO_VORTEX),
      new Spark(Constants.ShooterHardware.ANGLE_MOTOR_ID, MotorKind.NEO_VORTEX),
      new Spark(Constants.ShooterHardware.INDEXER_MOTOR_ID, MotorKind.NEO)
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
    m_desiredShooterState = normalizeState(state);
    m_topFlywheelMotor.set(m_desiredShooterState.speed.in(Units.MetersPerSecond), ControlType.kVelocity);
    // m_angleMotor.smoothMotion(
    //   m_desiredShooterState.angle.minus(SHOOTER_ANGLE_OFFSET).in(Units.Radians),
    //   m_angleConstraint,
    //   this::angleFFCalculator
    // );
  }

  /**
   * Normalize shooter state to be within valid values
   * @param state Desired state
   * @return Valid shooter state
   */
  private State normalizeState(State state) {
    return new State(
      Units.MetersPerSecond.of(MathUtil.clamp(
        state.speed.in(Units.MetersPerSecond),
        -MAX_FLYWHEEL_SPEED.in(Units.MetersPerSecond),
        +MAX_FLYWHEEL_SPEED.in(Units.MetersPerSecond)
      )),
      Units.Radians.of(MathUtil.clamp(
        state.angle.in(Units.Radians),
        SHOOTER_ANGLE_OFFSET.in(Units.Radians),
        m_angleConfig.getUpperLimit() + SHOOTER_ANGLE_OFFSET.in(Units.Radians)
      ))
    );
  }

  /**
   * Reset shooter state
   */
  private void resetState() {
    setState(new State(ZERO_FLYWHEEL_SPEED, SHOOTER_ANGLE_OFFSET));
  }

  /**
   * Get current shooter state
   * @return Current shooter state
   */
  private State getCurrentState() {
    return new State(
      Units.MetersPerSecond.of(m_topFlywheelMotor.getInputs().encoderVelocity),
      Units.Radians.of(m_angleMotor.getInputs().absoluteEncoderPosition).plus(SHOOTER_ANGLE_OFFSET)
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
   * Feed forward calculator for shooter angle
   * @param state Current motion profile state
   * @return Feed forward voltage to apply
   */
  private double angleFFCalculator(TrapezoidProfile.State state) {
    return m_angleFF.calculate(state.velocity, state.position);
  }

  /**
   * Get distance to target, clamped to maximum shooting distance
   * @return Distance to target
   */
  private Measure<Distance> getTargetDistance() {
    return Units.Meters.of(
      MathUtil.clamp(
        m_poseSupplier.get().getTranslation().getDistance(m_targetSupplier.get().getSecond()),
        MIN_SHOOTING_DISTANCE.in(Units.Meters),
        MAX_SHOOTING_DISTANCE.in(Units.Meters)
      )
    );
  }

  /**
   * Check if shooter has reached desired state and is ready
   * @return True if ready
   */
  private boolean isReady() {
    return m_angleMotor.isSmoothMotionFinished() &&
      Math.abs(m_topFlywheelMotor.getInputs().encoderVelocity - m_desiredShooterState.speed.in(Units.MetersPerSecond)) < m_flywheelConfig.getTolerance();
  }

  /**
   * Feed game piece to flywheels
   */
  private void feedStart() {
    m_indexerMotor.set(+INDEXER_SPEED.in(Units.Percent));
  }

  private void feedReverse() {
    m_indexerMotor.set(-INDEXER_SPEED.in(Units.Percent));
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

    var currentState = getCurrentState();
    Logger.recordOutput(getName() + MECHANISM_2D_LOG_ENTRY, m_mechanism2d);
    Logger.recordOutput(getName() + SHOOTER_STATE_FLYWHEEL_SPEED, currentState.speed.in(Units.MetersPerSecond));
    Logger.recordOutput(getName() + SHOOTER_STATE_ANGLE_DEGREES, currentState.angle.in(Units.Degrees));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run in simulation
    m_topFlywheelMotor.getInputs().encoderVelocity = m_desiredShooterState.speed.in(Units.MetersPerSecond);

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
    return startEnd(
      () -> {
        m_indexerMotor.enableForwardLimitSwitch();
        feedStart();
      },
      () -> {
        m_indexerMotor.disableForwardLimitSwitch();
        feedStop();
      }
    );
  }

  /**
   * Intake a game piece from the ground intake to be later fed to the shooter with no limit switches
   * @return Command to feed through the shooter
   */
  public Command feedCommand() {
    return startEnd(() -> feedStart(), () -> feedStop());
  }

  /**
   * Reverse note from shoter into intake
   * @return Command to outtake note in shooter
   */
  public Command outtakeCommand() {
    return startEnd(
      () -> feedReverse(),
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
        if (isReady()) feedStart();
        else feedStop();
      },
      () -> {
        feedStop();
        resetState();
      }
    ).beforeStarting(() -> setState(stateSupplier.get()), this);
  }

  /**
   * Shoot automatically based on current location
   * @param isAimed Is robot aimed at target
   * @param override Shoot even if target tag is not visible
   * @return Command to automatically shoot note
   */
  public Command shootCommand(BooleanSupplier isAimed, boolean override) {
    return runEnd(
      () -> {
        setState(getAutomaticState());
        if (RobotBase.isSimulation() | isReady()
            && isAimed.getAsBoolean()
            && VisionSubsystem.getInstance().getVisibleTagIDs().contains(m_targetSupplier.get().getFirst()) | override)
          feedStart();
        else feedStop();
      },
      () -> {
        feedStop();
        resetState();
      }
    );
  }

 /**
   * Shoot automatically based on current location, checking if target tag is visible
   * @param isAimed Is robot aimed at target
   * @return Command to automatically shoot note
   */
  public Command shootCommand(BooleanSupplier isAimed) {
    return shootCommand(isAimed, false);
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
   * Whether a game piece is in the indexer
   * @return The value of the indexer motor's forward limit switch
   */
  public boolean isObjectPresent() {
    return m_indexerMotor.getInputs().forwardLimitSwitch;
  }

  @Override
  public void close() {
    m_topFlywheelMotor.close();
    m_bottomFlywheelMotor.close();
    m_angleMotor.close();
    m_indexerMotor.close();
  }
}
