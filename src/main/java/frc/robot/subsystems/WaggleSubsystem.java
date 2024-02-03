// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.FeedbackSensor;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class WaggleSubsystem extends SubsystemBase {
  Spark m_motor;
  SparkPIDConfig m_config;
  Constraints m_constraint;

  /** Creates a new wiggleStick. */
  public WaggleSubsystem(SparkPIDConfig config, Constraints constraint) {
    m_motor = new Spark(new Spark.ID("wiggleStick", 20), MotorKind.NEO);
    m_config = config;
    m_constraint = constraint;

    double conversionFactor = 1.0;
    m_motor.initializeSparkPID(config, FeedbackSensor.NEO_ENCODER);
    m_motor.setPositionConversionFactor(FeedbackSensor.NEO_ENCODER, conversionFactor);
    m_motor.setVelocityConversionFactor(FeedbackSensor.NEO_ENCODER, conversionFactor);

    m_motor.enablePIDWrapping(0.0, 15.0);
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.resetEncoder();
  }

  public void setPosition(double position) {
    m_motor.smoothMotion(position, m_constraint);
  }

  public double getPosition() {
    return m_motor.getInputs().encoderPosition;
  }

  public Command setPositionCommand(double position) {
    return runOnce(() -> setPosition(position));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_motor.periodic();
  }
}
