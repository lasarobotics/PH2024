// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.SystemState;

public class IntakeSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware(Spark rollerMotor) {}

  public enum State implements SystemState {
    IDLE {
      @Override
      public void initialize() {
        s_instance.stop();
      }

      @Override
      public State nextState() {
        if (s_intakeButton.getAsBoolean() && !s_outtakeButton.getAsBoolean()) return INTAKING;
        if (s_outtakeButton.getAsBoolean() && !s_intakeButton.getAsBoolean()) return OUTTAKING;
        return this;
      }
    },
    INTAKING {
      @Override
      public void initialize() {
        s_instance.intake();
      }

      @Override
      public State nextState() {
        if (!s_intakeButton.getAsBoolean()) return IDLE;
        return this;
      }
    },
    OUTTAKING {
      @Override
      public void initialize() {
        s_instance.outtake();
      }

      @Override
      public State nextState() {
        if (!s_outtakeButton.getAsBoolean()) return IDLE;
        return this;
      }
    };
  }


  private static IntakeSubsystem s_instance;
  private static Trigger s_intakeButton = new Trigger(() -> false);
  private static Trigger s_outtakeButton = new Trigger(() -> false);

  private final Measure<Dimensionless> m_rollerVelocity;

  private final Spark m_rollerMotor;

  /** Creates a new IntakeSubsystem. */
  private IntakeSubsystem(Hardware intakeHardware, Measure<Dimensionless> rollerVelocity) {
    super(State.IDLE);
    this.m_rollerMotor = intakeHardware.rollerMotor;
    this.m_rollerVelocity = rollerVelocity;

    // Reset motor to defaults
    m_rollerMotor.restoreFactoryDefaults();

    // Set idle mode
    m_rollerMotor.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Get an instance of IntakeSubsystem
   * <p>
   * Will only return an instance once, subsequent calls will return null.
   * @param intakeHardware Necessary hardware for this subsystem
   * @return Subsystem instance
   */
  public static IntakeSubsystem getInstance(Hardware intakeHardware) {
    if (s_instance == null) {
      s_instance = new IntakeSubsystem(intakeHardware, Constants.Intake.ROLLER_VELOCITY);
      return s_instance;
    } else return null;
  }

  /**
   * Initialize hardware devices for intake subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware intakeHardware = new Hardware(
      new Spark(Constants.IntakeHardware.ROLLER_MOTOR_ID, MotorKind.NEO_VORTEX)
    );
    return intakeHardware;
  }

  /**
   * Intake game object
   */
  private void intake() {
    m_rollerMotor.set(+m_rollerVelocity.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Outtake game object
   */
  private void outtake() {
    m_rollerMotor.set(-m_rollerVelocity.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Stop intake roller
   */
  private void stop() {
    m_rollerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(getName() + "/State", getState().toString());
  }

  /**
   * Set intake button
   * @param intakeButton Button to use
   */
  public void bindIntakeButton(Trigger intakeButton) {
    s_intakeButton = intakeButton;
  }

  /**
   * Set outtake button
   * @param outtakeButton Button to use
   */
  public void bindOuttakeButton(Trigger outtakeButton) {
    s_outtakeButton = outtakeButton;
  }

  @Override
  public void close() {
    m_rollerMotor.close();
    s_instance = null;
  }
}
