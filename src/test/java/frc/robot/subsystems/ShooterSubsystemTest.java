// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class ShooterSubsystemTest {
  private final double DELTA = 5e-3;
  private ShooterSubsystem m_shooterSubsystem;
  private ShooterSubsystem.Hardware m_shooterHardware;


  private Spark m_topFlywheelMotor;
  private Spark m_bottomFlywheelMotor;
  private Spark m_angleMotor;
  private Spark m_indexerMotor;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices

    m_topFlywheelMotor = mock(Spark.class);
    m_bottomFlywheelMotor = mock(Spark.class);
    m_angleMotor = mock(Spark.class);
    m_indexerMotor = mock(Spark.class);

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
    m_shooterHardware = new ShooterSubsystem.Hardware(m_topFlywheelMotor, m_bottomFlywheelMotor, m_angleMotor, m_indexerMotor);

    m_shooterSubsystem = new ShooterSubsystem(
      m_shooterHardware,
      Constants.Shooter.FLYWHEEL_CONFIG,
      Constants.Shooter.ANGLE_CONFIG,
      Constants.Shooter.ANGLE_FF,
      Constants.Shooter.ANGLE_MOTION_CONSTRAINT,
      Constants.Shooter.FLYWHEEL_DIAMETER,
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
    var state = new ShooterSubsystem.State(Units.MetersPerSecond.of(0.0), Units.Degrees.of(30.0));
    var command = m_shooterSubsystem.shootManualCommand(() -> state);
    command.initialize();

    // Verify that motors are being driven with expected values
    verify(m_topFlywheelMotor, times(1)).set(AdditionalMatchers.eq(state.speed.in(Units.MetersPerSecond), DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_angleMotor, times(1)).smoothMotion(
      AdditionalMatchers.eq(state.angle.minus(ShooterSubsystem.SHOOTER_ANGLE_OFFSET).in(Units.Radians), DELTA),
      ArgumentMatchers.eq(Constants.Shooter.ANGLE_MOTION_CONSTRAINT),
      ArgumentMatchers.any()
    );
  }
}
