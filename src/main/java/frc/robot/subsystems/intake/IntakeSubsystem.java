// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.FeedbackSensor;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public static class Hardware {
    private Spark rollerMotor;

    public Hardware(Spark rollerMotor) {
      this.rollerMotor = rollerMotor;
    }
  }

  private Spark m_rollerMotor;

  private final Measure<Velocity<Angle>> ROLLER_VELOCITY;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(Hardware intakeHardware, SparkPIDConfig config, Measure<Velocity<Angle>> rollerVelocity) {
    this.m_rollerMotor = intakeHardware.rollerMotor;
    m_rollerMotor.initializeSparkPID(config, FeedbackSensor.NEO_ENCODER);
    
    ROLLER_VELOCITY = rollerVelocity;
  }
  
  /**
   * Initialize hardware devices for intake subsystem
   * 
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware intakeHardware = new Hardware(
      new Spark(Constants.IntakeHardware.ROLLER_MOTOR_ID, MotorKind.NEO)
    );
    return intakeHardware;
  }

  // Tells the robot to intake
  private void intake() {
    m_rollerMotor.set(ROLLER_VELOCITY.in(Units.RPM), ControlType.kVelocity);
  }

  // Tells the robot to outtake
  private void outtake() {
    m_rollerMotor.set(-ROLLER_VELOCITY.in(Units.RPM), ControlType.kVelocity);
  }

  // Stop the robot
  private void stop() {
    m_rollerMotor.stopMotor();;
  }

  @Override
  public void periodic() {
    m_rollerMotor.periodic();
  }
  
  /**
   * Intake game piece from ground
   * @return Command to run the roller motor
   */
  public Command intakeCommand() {
    return startEnd(() -> intake(), () -> stop());
  }

  /**
   * Spit out game piece from intake
   * @return Command to run the roller motor in the reverse direction
   */
  public Command outtakeCommand() {
    return startEnd(() -> outtake(), () -> stop());
  }

  /**
   * Whether a game piece is in the intake
   * @return The value of the roller motor's forward limit switch
   */
  public boolean isObjectPresent() {
    return m_rollerMotor.getInputs().forwardLimitSwitch;
  }
}
