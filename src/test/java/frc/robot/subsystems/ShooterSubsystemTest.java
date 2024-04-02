// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkInputsAutoLogged;
import org.lasarobotics.led.LEDStrip;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class ShooterSubsystemTest {
  private final double DELTA = 5e-3;
  private static final Measure<Dimensionless> INDEXER_SPEED = Units.Percent.of(100.0);
  private static final Measure<Dimensionless> INDEXER_SLOW_SPEED = Units.Percent.of(4.0);

  private ShooterSubsystem m_shooterSubsystem;
  private ShooterSubsystem.Hardware m_shooterHardware;

  private Spark m_topFlywheelMotor;
  private Spark m_bottomFlywheelMotor;
  private Spark m_angleMotor;
  private Spark m_indexerMotor;
  private LEDStrip m_ledStrip;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_topFlywheelMotor = mock(Spark.class);
    m_bottomFlywheelMotor = mock(Spark.class);
    m_angleMotor = mock(Spark.class);
    m_indexerMotor = mock(Spark.class);
    m_ledStrip = mock(LEDStrip.class);

    SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();
    when(m_topFlywheelMotor.getInputs()).thenReturn(sparkInputs);
    when(m_angleMotor.getInputs()).thenReturn(sparkInputs);
    when(m_indexerMotor.getInputs()).thenReturn(sparkInputs);

    when(m_topFlywheelMotor.getKind()).thenReturn(MotorKind.NEO);
    when(m_bottomFlywheelMotor.getKind()).thenReturn(MotorKind.NEO);
    when(m_angleMotor.getKind()).thenReturn(MotorKind.NEO);
    when(m_indexerMotor.getKind()).thenReturn(MotorKind.NEO);

    when(m_topFlywheelMotor.getID()).thenReturn(Constants.DriveHardware.LEFT_FRONT_DRIVE_MOTOR_ID);
    when(m_bottomFlywheelMotor.getID()).thenReturn(Constants.DriveHardware.LEFT_FRONT_ROTATE_MOTOR_ID);
    when(m_angleMotor.getID()).thenReturn(Constants.DriveHardware.RIGHT_FRONT_DRIVE_MOTOR_ID);
    when(m_indexerMotor.getID()).thenReturn(Constants.DriveHardware.RIGHT_FRONT_ROTATE_MOTOR_ID);

    // Create hardware object using mock devices
    m_shooterHardware = new ShooterSubsystem.Hardware(m_topFlywheelMotor, m_bottomFlywheelMotor, m_angleMotor, m_indexerMotor, m_ledStrip);

    m_shooterSubsystem = new ShooterSubsystem(
      m_shooterHardware,
      Constants.Shooter.FLYWHEEL_CONFIG,
      Constants.Shooter.ANGLE_CONFIG,
      Constants.Shooter.ANGLE_MOTION_CONSTRAINT,
      Constants.Shooter.TOP_FLYWHEEL_DIAMETER,
      Constants.Shooter.BOTTOM_FLYWHEEL_DIAMETER,
      Constants.Shooter.SHOOTER_MAP,
      () -> new Pose2d(),
      () -> Constants.Field.BLUE_SPEAKER
    );
  }

  @AfterEach
  public void close() {
    m_shooterSubsystem.close();
    m_shooterSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can set shooter state")
  public void setState() {
    // Hardcode sensor values
    SparkInputsAutoLogged inputs = new SparkInputsAutoLogged();
    when(m_topFlywheelMotor.getInputs()).thenReturn(inputs);
    when(m_angleMotor.getInputs()).thenReturn(inputs);

    // Try to set shooter state
    var state = new ShooterSubsystem.State(Units.MetersPerSecond.of(+15.0), Units.Degrees.of(30.0));
    var command = m_shooterSubsystem.shootManualCommand(() -> state);
    command.initialize();

    // Verify that motors are being driven with expected values
    verify(m_topFlywheelMotor, times(1)).set(AdditionalMatchers.eq(state.speed.in(Units.MetersPerSecond), DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_angleMotor, times(1)).set(
      AdditionalMatchers.eq(state.angle.in(Units.Radians), DELTA),
      ArgumentMatchers.eq(ControlType.kPosition)
    );
  }

  @Test
  @Order(2)
  @DisplayName("Test if the shooter ignores illegal high angles")
  public void setHighAngle() {
    // Hardcode sensor values
    SparkInputsAutoLogged inputs = new SparkInputsAutoLogged();
    when(m_topFlywheelMotor.getInputs()).thenReturn(inputs);
    when(m_angleMotor.getInputs()).thenReturn(inputs);

    // Try to set shooter state
    Measure<Angle> illegalHighAngle = Units.Radians.of(Constants.Shooter.ANGLE_CONFIG.getUpperLimit()).plus(Units.Degrees.of(30.0));
    var state = new ShooterSubsystem.State(Units.MetersPerSecond.of(+15.0), illegalHighAngle);
    var command = m_shooterSubsystem.shootManualCommand(() -> state);
    command.initialize();

    // Verify that motors are being driven with expected values
    verify(m_topFlywheelMotor, times(1)).set(AdditionalMatchers.eq(state.speed.in(Units.MetersPerSecond), DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_angleMotor, times(1)).set(
      AdditionalMatchers.eq(Constants.Shooter.ANGLE_CONFIG.getUpperLimit(), DELTA),
      ArgumentMatchers.eq(ControlType.kPosition)
    );
  }

  @Test
  @Order(3)
  @DisplayName("Test if the shooter ignores illegal low angles")
  public void setLowAngle() {
    // Hardcode sensor values
    SparkInputsAutoLogged inputs = new SparkInputsAutoLogged();
    when(m_topFlywheelMotor.getInputs()).thenReturn(inputs);
    when(m_angleMotor.getInputs()).thenReturn(inputs);

    // Try to set shooter state
    Measure<Angle> illegalLowAngle = Units.Radians.of(Constants.Shooter.ANGLE_CONFIG.getLowerLimit()).minus(Units.Degrees.of(30.0));
    var state = new ShooterSubsystem.State(Units.MetersPerSecond.of(+15.0), illegalLowAngle);
    var command = m_shooterSubsystem.shootManualCommand(() -> state);
    command.initialize();

    // Verify that motors are being driven with expected values
    verify(m_topFlywheelMotor, times(1)).set(AdditionalMatchers.eq(state.speed.in(Units.MetersPerSecond), DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_angleMotor, times(1)).set(
      AdditionalMatchers.eq(Constants.Shooter.ANGLE_CONFIG.getLowerLimit(), DELTA),
      ArgumentMatchers.eq(ControlType.kPosition)
    );
  }

  @Test
  @Order(4)
  @DisplayName("Test if the shooter runs the indexer when the shooter is at the correct state")
  public void runIndexer() {
    // Hardcode sensor values
    SparkInputsAutoLogged flywheelInputs = new SparkInputsAutoLogged();
    SparkInputsAutoLogged angleInputs = new SparkInputsAutoLogged();

    // Try to set shooter state
    var state = new ShooterSubsystem.State(Units.MetersPerSecond.of(+15.0), Units.Degrees.of(30.0));
    var command = m_shooterSubsystem.shootManualCommand(() -> state);
    command.initialize();

    // Set the shooter to be at the correct state
    flywheelInputs.encoderVelocity = state.speed.in(Units.MetersPerSecond);
    angleInputs.absoluteEncoderPosition = state.angle.in(Units.Radians);
    when(m_topFlywheelMotor.getInputs()).thenReturn(flywheelInputs);
    when(m_bottomFlywheelMotor.getInputs()).thenReturn(flywheelInputs);
    when(m_angleMotor.isSmoothMotionFinished()).thenReturn(true);
    when(m_angleMotor.getInputs()).thenReturn(angleInputs);

    // Execute the command when the state is ready
    command.execute();
    Timer.delay(0.1);
    command.execute();

    // Verify that motors are being driven with expected values
    verify(m_indexerMotor, times(1)).set(AdditionalMatchers.eq(+INDEXER_SPEED.in(Units.Percent), DELTA));
  }

  @Test
  @Order(5)
  @DisplayName("Test if the indexer runs in the correct direction when intaking via the flywheels (from source)")
  public void reverseIndexer() {
    // Hardcode sensor values
    SparkInputsAutoLogged inputs = new SparkInputsAutoLogged();
    when(m_topFlywheelMotor.getInputs()).thenReturn(inputs);
    when(m_angleMotor.getInputs()).thenReturn(inputs);

    // Run the source intake command
    var command = m_shooterSubsystem.sourceIntakeCommand();
    command.initialize();
    command.execute();

    // Verify that motors are being driven with expected values in the reverse direction
    verify(m_indexerMotor, times(1)).set(AdditionalMatchers.eq(-INDEXER_SLOW_SPEED.in(Units.Percent), DELTA));
  }

  @Test
  @Order(6)
  @DisplayName("Test if shooter stops the indexer when the beam break is triggered (intake from source)")
  public void stopIndexerSource() {
    // Hardcode sensor values
    SparkInputsAutoLogged inputs = new SparkInputsAutoLogged();
    when(m_topFlywheelMotor.getInputs()).thenReturn(inputs);
    when(m_angleMotor.getInputs()).thenReturn(inputs);

    // Enable the indexer beam break
    SparkInputsAutoLogged indexerInputs = new SparkInputsAutoLogged();
    indexerInputs.forwardLimitSwitch = true;
    indexerInputs.reverseLimitSwitch = true;
    when(m_indexerMotor.getInputs()).thenReturn(indexerInputs);

    // Run the source intake command
    var command = m_shooterSubsystem.sourceIntakeCommand();
    command.initialize();
    command.execute();

    // Verify that motor is being stopped
    assertEquals(true, command.isFinished());
  }

  @Test
  @Order(7)
  @DisplayName("Test if shooter stops the indexer when the beam break is triggered (intake from ground)")
  public void stopIndexerGround() {
    // Hardcode sensor values
    SparkInputsAutoLogged inputs = new SparkInputsAutoLogged();
    when(m_topFlywheelMotor.getInputs()).thenReturn(inputs);
    when(m_angleMotor.getInputs()).thenReturn(inputs);

    // Enable the indexer beam break
    SparkInputsAutoLogged indexerInputs = new SparkInputsAutoLogged();
    indexerInputs.forwardLimitSwitch = true;
    indexerInputs.reverseLimitSwitch = true;
    when(m_indexerMotor.getInputs()).thenReturn(indexerInputs);

    // Run the source intake command
    var command = m_shooterSubsystem.intakeCommand();
    command.initialize();
    command.execute();

    // Verify that motor is being stopped
    assertEquals(true, command.isFinished());
  }
}
