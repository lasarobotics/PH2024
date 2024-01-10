// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

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

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(Hardware intakeHardware) {
    this.m_rollerMotor = intakeHardware.rollerMotor;
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
  private void intake(double speed) {
    m_rollerMotor.set(speed, ControlType.kDutyCycle, 0.0, ArbFFUnits.kPercentOut);
  }

  public Command intakeCommand(DoubleSupplier speed) {
    return startEnd(() -> intake(speed.getAsDouble()), () -> stop());
  }

  // Tells the robot to outtake
  private void outtake(double speed) {
    m_rollerMotor.set(-speed, ControlType.kDutyCycle, 0.0, ArbFFUnits.kPercentOut);
  }

  public Command outtakeCommand(DoubleSupplier speed) {
    return startEnd(() -> outtake(speed.getAsDouble()), () -> stop());
  }

  // Stop the robot
  private void stop() {
    m_rollerMotor.stopMotor();;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
