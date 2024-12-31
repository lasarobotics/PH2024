// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

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

public class ClimberSubsystem extends StateMachine implements AutoCloseable {
  public static record Hardware(Spark lClimberMotor, Spark rClimberMotor) {}
  public enum State implements SystemState {
  IDLE {
    @Override
    public void initialize() {
      s_instance.stop();
    }

    @Override
    public State nextState() {
      if (s_climbButton.getAsBoolean() && !s_retractButton.getAsBoolean()) return RELEASING;
      if (s_retractButton.getAsBoolean() && !s_climbButton.getAsBoolean()) return RETRACTING;
      return this;
    }
  },
  RELEASING {
    @Override
    public void initialize() {
      s_instance.runClimber();
    }

    @Override
    public State nextState() {
      if (!s_climbButton.getAsBoolean()) return IDLE;
      return this;
    }
  },
  RETRACTING {
    @Override
    public void initialize() {
      s_instance.retractClimber();
    }

    @Override
    public State nextState() {
      if (!s_retractButton.getAsBoolean()) return IDLE;
      return this;
    }
  };
}



  

  private static ClimberSubsystem s_instance;
  private static Trigger s_climbButton = new Trigger(() -> false);
  private static Trigger s_retractButton = new Trigger(() -> false);

  private final Measure<Dimensionless> CLIMBER_VELOCITY;

  private Spark m_lClimberMotor;
  private Spark m_rClimberMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(Hardware climberHardware, Measure<Dimensionless> climberVelocity) {
    super(State.IDLE);
    this.m_lClimberMotor = climberHardware.lClimberMotor;
    this.m_rClimberMotor = climberHardware.rClimberMotor;
    CLIMBER_VELOCITY = climberVelocity;

    m_lClimberMotor.restoreFactoryDefaults();
    m_rClimberMotor.restoreFactoryDefaults();

    m_rClimberMotor.setInverted(true);

    m_lClimberMotor.enableReverseLimitSwitch();
    m_rClimberMotor.enableReverseLimitSwitch();

    m_lClimberMotor.setIdleMode(IdleMode.kBrake);
    m_rClimberMotor.setIdleMode(IdleMode.kBrake);

  }

  /**
   * Initialize hardware devices for climber subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware climberHardware = new Hardware(
      new Spark(Constants.ClimberHardware.LEFT_CLIMBER_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.ClimberHardware.RIGHT_CLIMBER_MOTOR_ID, MotorKind.NEO)
    );

    return climberHardware;
  }

  /**
   * Runs the climber during a match
   */
  private void runClimber() {
    m_lClimberMotor.set(CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
    m_rClimberMotor.set(CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
  }

  private void retractClimber() {
    m_lClimberMotor.set(-CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
    m_rClimberMotor.set(-CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
  }

  /**
   * Stop both motors
   */
  private void stop() {
    m_lClimberMotor.stopMotor();
    m_rClimberMotor.stopMotor();
  }


    /**
   * Set climb button
   * @param climbButtonTrigger Button to use
   */
  public void bindClimbButton(Trigger climbButtonTrigger) {
    s_climbButton = climbButtonTrigger;
  }

  /**
   * Set retract button
   * @param retractButtonTrigger Button to use
   */
  public void bindRetractButton(Trigger retractButtonTrigger) {
    s_retractButton = retractButtonTrigger;
  }

  @Override
  public void periodic() {
    Logger.recordOutput(getName() + "/State", getState().toString());
  }

  @Override
  public void close() {
    m_lClimberMotor.close();
    m_rClimberMotor.close();
    s_instance = null;
  }
}
