// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.FeedbackSensor;
import org.lasarobotics.hardware.revrobotics.Spark.ID;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class wiggleStick extends SubsystemBase {
  Spark m_motor;
  SparkPIDConfig m_PidConfig;
  Constraints m_constraint;

  /** Creates a new wiggleStick. */
  public wiggleStick(SparkPIDConfig config, Constraints constraint) {
    m_motor = new Spark(new ID("wiggleStick", 20), MotorKind.NEO);
    m_PidConfig = config;
    m_constraint = constraint;

    m_motor.initializeSparkPID(config, FeedbackSensor.NEO_ENCODER);
  }

  public void setPosition(double position) {
    m_motor.smoothMotion(position, m_constraint);
  }

  public double getPosition() {
    return m_motor.getInputs().encoderPosition;
  }

  public Command setPositionCommand(DoubleSupplier position) {
    return runOnce(() -> setPosition(position.getAsDouble()));
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_motor.periodic();
  }
}
