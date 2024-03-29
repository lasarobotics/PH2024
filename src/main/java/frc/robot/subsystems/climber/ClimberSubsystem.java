// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  public static class Hardware {
    private Spark lClimberMotor;
    private Spark rClimberMotor;

    public Hardware(Spark lClimberMotor, Spark rClimberMotor) {
      this.lClimberMotor = lClimberMotor;
      this.rClimberMotor = rClimberMotor;
    }
  }

  private Spark m_lClimberMotor;
  private Spark m_rClimberMotor;

  private Measure<Dimensionless> CLIMBER_VELOCITY;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(Hardware climberHardware, Measure<Dimensionless> climberVelocity) {
    this.m_lClimberMotor = climberHardware.lClimberMotor;
    this.m_rClimberMotor = climberHardware.rClimberMotor;

    m_lClimberMotor.setInverted(true);
    m_rClimberMotor.setInverted(true);


    m_lClimberMotor.enableReverseLimitSwitch();
    m_rClimberMotor.enableReverseLimitSwitch();

    m_lClimberMotor.setIdleMode(IdleMode.kBrake);
    m_rClimberMotor.setIdleMode(IdleMode.kBrake);

    CLIMBER_VELOCITY = climberVelocity;
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

  // Raises the arms
  private void raiseClimber() {
    m_lClimberMotor.set(CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
    m_rClimberMotor.set(CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
  }

  // Lowers the arms
  private void lowerClimber() {
    m_lClimberMotor.set(-CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
    m_rClimberMotor.set(-CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
  }

  // Stop both motors
  private void stop() {
    m_lClimberMotor.stopMotor();
    m_rClimberMotor.stopMotor();
  }

  /**
   * Raise climber arms
   * @return Command to run the climber motors
   */
  public Command raiseClimberCommand() {
    return runEnd(() -> raiseClimber(), () -> stop());
  }

  /**
   * Lower climber arms
   * @return Command to run the climber motors in the reverse direction
   */
  public Command lowerClimberCommand() {
    return runEnd(() -> lowerClimber(), () -> stop());
  }

  public Command lowerLeftCommand() {
    return runEnd(() -> m_lClimberMotor.set(-CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle), () -> stop());
  }
  public Command raiseLeftCommand() {
    return runEnd(() -> m_lClimberMotor.set(CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle), () -> stop());
  }
  public Command lowerRightCommand() {
    return runEnd(() -> m_rClimberMotor.set(-CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle), () -> stop());
  }
  public Command raiseRightCommand() {
    return runEnd(() -> m_rClimberMotor.set(CLIMBER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle), () -> stop());
  }

  @Override
  public void periodic() {
    m_lClimberMotor.periodic();
    m_rClimberMotor.periodic();
    // m_rClimberMotor.stopMotor();

    CLIMBER_VELOCITY = Units.Percent.of(SmartDashboard.getNumber("climber power", 0));
  }
}
