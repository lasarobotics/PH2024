// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;

/** Add your docs here. */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class IntakeSubsystemTest {
  private final double DELTA = 1e-3;

  private Spark m_rollerMotor;

  private IntakeSubsystem m_intakeSubsystem;
  private IntakeSubsystem.Hardware m_intakeHardware;
  private Command m_stateMachine;

  private boolean m_intakeButton = false;
  private boolean m_outtakeButton = false;

  @BeforeEach
  public void setup() {
    // Create mock hardware device
    m_rollerMotor = mock(Spark.class);

    // Create hardware object using mock device
    m_intakeHardware = new IntakeSubsystem.Hardware(m_rollerMotor);

    // Instantiate subsystem
    m_intakeSubsystem = IntakeSubsystem.getInstance(m_intakeHardware);

    // Bind buttons
    m_intakeSubsystem.bindIntakeButton(new Trigger(() -> m_intakeButton));
    m_intakeSubsystem.bindOuttakeButton(new Trigger(() -> m_outtakeButton));

    // Get state machine command and initialize default state
    m_stateMachine = m_intakeSubsystem.getDefaultCommand();
    m_stateMachine.initialize();

    // Make sure intake starts at idle
    assertEquals(IntakeSubsystem.State.IDLE, m_intakeSubsystem.getState());
  }

  @AfterEach
  public void close() {
    m_intakeSubsystem.close();
    m_intakeSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can intake")
  public void intake() {
    // Press button and iterate state machine
    m_intakeButton = true;
    m_stateMachine.execute();
    m_stateMachine.execute();

    // Verify state change and motors are being driven with expected values
    assertEquals(IntakeSubsystem.State.INTAKING, m_intakeSubsystem.getState());
    verify(m_rollerMotor, times(1)).set(
      AdditionalMatchers.eq(Constants.Intake.ROLLER_VELOCITY.in(Units.Percent), DELTA),
      ArgumentMatchers.eq(ControlType.kDutyCycle)
    );

    // Release button and iterate state machine
    m_intakeButton = false;
    m_stateMachine.execute();
    m_stateMachine.execute();

    // Verify state change and motors are stopped
    assertEquals(IntakeSubsystem.State.IDLE, m_intakeSubsystem.getState());
    verify(m_rollerMotor, times(2)).stopMotor();
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can outtake")
  public void outtake() {
    // Press button and iterate state machine
    m_outtakeButton = true;
    m_stateMachine.execute();
    m_stateMachine.execute();

    // Verify state change and motors are being driven with expected values
    assertEquals(IntakeSubsystem.State.OUTTAKING, m_intakeSubsystem.getState());
    verify(m_rollerMotor, times(1)).set(
      AdditionalMatchers.eq(Constants.Intake.ROLLER_VELOCITY.negate().in(Units.Percent), DELTA),
      ArgumentMatchers.eq(ControlType.kDutyCycle)
    );

    // Verify state change and iterate state machine
    m_outtakeButton = false;
    m_stateMachine.execute();
    m_stateMachine.execute();

    // Verify state change and motors are stopped
    assertEquals(IntakeSubsystem.State.IDLE, m_intakeSubsystem.getState());
    verify(m_rollerMotor, times(2)).stopMotor();
  }
}
