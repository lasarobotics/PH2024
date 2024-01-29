// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.List;
import java.util.Map.Entry;
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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
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
  public static class ShooterState {
    public final Measure<Velocity<Distance>> shooterSpeed;
    public final Measure<Angle> shooterAngle;

    public ShooterState(Measure<Velocity<Distance>> shooterSpeed, Measure<Angle> shooterAngle) {
      this.shooterSpeed = shooterSpeed;
      this.shooterAngle = shooterAngle;
    }
  }

  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  private static final Measure<Angle> SHOOTER_ANGLE_OFFSET = Units.Degrees.of(25.0);
  private static final Measure<Velocity<Distance>> ZERO_FLYWHEEL_SPEED = Units.MetersPerSecond.of(0.0);
  private static final Measure<Dimensionless> INTAKE_SPEED = Units.Percent.of(50.0);
  private static final double EPSILON = 5.0;

  private final Measure<Distance> MIN_SHOOTING_DISTANCE = Units.Meters.of(0.0);
  private final Measure<Distance> MAX_SHOOTING_DISTANCE;

  private Spark m_topFlywheelMotor;
  private Spark m_bottomFlywheelMotor;
  private Spark m_angleMotor;
  private Spark m_indexerMotor;

  private ElevatorFeedforward m_angleFF;
  private TrapezoidProfile.Constraints m_angleConstraint;
  private Supplier<Pose2d> m_poseSupplier;
  private Supplier<Translation2d> m_targetSupplier;

  private ShooterState m_desiredShooterState;
  private PolynomialSplineFunction m_shooterAngleCurve;
  private PolynomialSplineFunction m_shooterFlywheelCurve;

  private Mechanism2d m_mechanism2d;
  private MechanismLigament2d m_simShooterJoint;
  private TrapezoidProfile.State m_simShooterAngleState;

  /**
   * Create an instance of ShooterSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param shooterHardware Hardware devices required by shooter
   */
  public ShooterSubsystem(Hardware shooterHardware, SparkPIDConfig flywheelConfig, SparkPIDConfig angleConfig,
                          FFConstants angleFF, TrapezoidProfile.Constraints angleConstraint, Measure<Distance> flywheelDiameter,
                          List<Entry<Measure<Distance>, ShooterState>> shooterMap,
                          Supplier<Pose2d> poseSupplier, Supplier<Translation2d> targetSupplier) {
    setSubsystem(getClass().getSimpleName());
    this.m_topFlywheelMotor = shooterHardware.topFlywheelMotor;
    this.m_bottomFlywheelMotor = shooterHardware.bottomFlywheelMotor;
    this.m_angleMotor = shooterHardware.angleMotor;
    this.m_indexerMotor = shooterHardware.indexerMotor;
    this.m_angleFF = new ElevatorFeedforward(angleFF.kS, angleFF.kG, angleFF.kV, angleFF.kA);
    this.m_angleConstraint = angleConstraint;
    this.m_poseSupplier = poseSupplier;
    this.m_targetSupplier = targetSupplier;

    // Slave bottom flywheel motor to top
    m_bottomFlywheelMotor.follow(m_topFlywheelMotor);

    // Initialize PID
    m_topFlywheelMotor.initializeSparkPID(flywheelConfig, FeedbackSensor.NEO_ENCODER);
    m_angleMotor.initializeSparkPID(angleConfig, FeedbackSensor.THROUGH_BORE_ENCODER, true, true);

    // Set flywheel conversion factor
    var flywheelConversionFactor = flywheelDiameter.in(Units.Meters) * Math.PI;
    m_topFlywheelMotor.setPositionConversionFactor(FeedbackSensor.NEO_ENCODER, flywheelConversionFactor);
    m_topFlywheelMotor.setVelocityConversionFactor(FeedbackSensor.NEO_ENCODER, flywheelConversionFactor / 60);

    // Set angle adjust conversion factor
    var angleConversionFactor = Math.PI * 2;
    m_angleMotor.setPositionConversionFactor(FeedbackSensor.NEO_ENCODER, angleConversionFactor);
    m_angleMotor.setVelocityConversionFactor(FeedbackSensor.NEO_ENCODER, angleConversionFactor / 60);

    // Initialize shooter state
    m_desiredShooterState = getCurrentShooterState();

    // Set maximum shooting distance
    MAX_SHOOTING_DISTANCE = shooterMap.get(shooterMap.size() - 1).getKey();

    // Initialize shooter curves
    initializeShooterCurves(shooterMap);

    // Initialize sim variables
    m_mechanism2d = new Mechanism2d(1.0, 1.0);
    m_simShooterJoint = m_mechanism2d.getRoot("shooter", 0.5, 0.33).append(new MechanismLigament2d("shooter", 0.4, 1.0));
    m_simShooterAngleState = new TrapezoidProfile.State();

    // Set commands
    setDefaultCommand(run(() ->
      setShooterState(
        new ShooterState(
          ZERO_FLYWHEEL_SPEED,
          Units.Radians.of(m_shooterAngleCurve.value(getTargetDistance().in(Units.Meters)))
        )
      )
    ));
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
  private void initializeShooterCurves(List<Entry<Measure<Distance>, ShooterState>> shooterMap) {
    double[] distances = new double[shooterMap.size()];
    double[] flywheelSpeeds = new double[shooterMap.size()];
    double[] angles = new double[shooterMap.size()];

    for (int i = 0; i < shooterMap.size(); i++) {
      distances[i] = shooterMap.get(i).getKey().in(Units.Meters);
      flywheelSpeeds[i] = shooterMap.get(i).getValue().shooterSpeed.in(Units.MetersPerSecond);
      angles[i] = shooterMap.get(i).getValue().shooterAngle.in(Units.Radians);
    }

    m_shooterFlywheelCurve = SPLINE_INTERPOLATOR.interpolate(distances, flywheelSpeeds);
    m_shooterAngleCurve = SPLINE_INTERPOLATOR.interpolate(distances, angles);
  }

  /**
   * Set shooter to desired state
   * @param state Desired shooter state
   */
  private void setShooterState(ShooterState state) {
    m_desiredShooterState = state;
    m_topFlywheelMotor.set(m_desiredShooterState.shooterSpeed.in(Units.MetersPerSecond), ControlType.kVelocity);
    m_angleMotor.smoothMotion(
      m_desiredShooterState.shooterAngle.minus(SHOOTER_ANGLE_OFFSET).in(Units.Radians),
      m_angleConstraint,
      this::angleFFCalculator
    );
  }

  /**
   * Get current shooter state
   * @return Current shooter state
   */
  private ShooterState getCurrentShooterState() {
    return new ShooterState(
      Units.MetersPerSecond.of(m_topFlywheelMotor.getInputs().encoderVelocity),
      Units.Radians.of(m_angleMotor.getInputs().absoluteEncoderPosition).plus(SHOOTER_ANGLE_OFFSET)
    );
  }

  /**
   * Feed forward calculator for shooter angle
   * @param state Current motion profile state
   * @return Feed forward voltage to apply
   */
  private double angleFFCalculator(TrapezoidProfile.State state) {
    return m_angleFF.calculate(state.velocity, state.position);
  }

  private Measure<Distance> getTargetDistance() {
    return Units.Meters.of(
      Math.min(m_poseSupplier.get().getTranslation().getDistance(m_targetSupplier.get()), MAX_SHOOTING_DISTANCE.in(Units.Meters))
    );
  }

  /**
   * Check if shooter has reached desired state and is ready
   * @return True if ready
   */
  private boolean isShooterReady() {
    return m_angleMotor.isSmoothMotionFinished() &&
      Math.abs(m_topFlywheelMotor.getInputs().encoderVelocity - m_desiredShooterState.shooterSpeed.in(Units.MetersPerSecond)) < EPSILON;
  }

  /**
   * Feed game piece to flywheels
   */
  private void feedStart() {
    m_indexerMotor.set(+INTAKE_SPEED.in(Units.Percent));
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

    Logger.recordOutput(getName() + "/Mechanism2d", m_mechanism2d);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run in simulation
    m_angleMotor.getInputs().absoluteEncoderPosition = m_simShooterAngleState.position;
    m_angleMotor.getInputs().absoluteEncoderVelocity = m_simShooterAngleState.velocity;

    m_simShooterAngleState = new TrapezoidProfile(m_angleConstraint).calculate(
      GlobalConstants.ROBOT_LOOP_PERIOD,
      new TrapezoidProfile.State(m_angleMotor.getInputs().absoluteEncoderPosition, m_angleMotor.getInputs().absoluteEncoderVelocity),
      new TrapezoidProfile.State(m_desiredShooterState.shooterAngle.in(Units.Radians), 0.0)
    );

    m_simShooterJoint.setAngle(Rotation2d.fromRadians(Math.PI - m_angleMotor.getInputs().absoluteEncoderPosition));
  }

  /**
   * Shoot by manually setting shooter state
   * @param stateSupplier Desired shooter state supplier
   * @return Command to control shooter manually
   */
  public Command shootManualCommand(Supplier<ShooterState> stateSupplier) {
    return Commands.parallel(
      runEnd(
        () -> {
          if (isShooterReady()) feedStart();
          else feedStop();
        },
        () -> {
          setShooterState(new ShooterState(ZERO_FLYWHEEL_SPEED, getCurrentShooterState().shooterAngle));
          feedStop();
        }
      ).beforeStarting(() -> setShooterState(stateSupplier.get()), this)
    );
  }

  /**
   * Move shooter to amp position
   * @return Command that prepares shooter for scoring in the amp
   */
  public Command prepareForAmpCommand() {
    return runEnd(
      () -> setShooterState(new ShooterState(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(90.0))),
      () -> setShooterState(new ShooterState(ZERO_FLYWHEEL_SPEED, getCurrentShooterState().shooterAngle))
    );
  }

  @Override
  public void close() {
    m_topFlywheelMotor.close();
    m_bottomFlywheelMotor.close();
    m_angleMotor.close();
    m_indexerMotor.close();
  }
}
