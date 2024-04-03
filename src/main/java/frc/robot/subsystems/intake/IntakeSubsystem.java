// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
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

  private final Measure<Dimensionless> ROLLER_VELOCITY;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(Hardware intakeHardware, Measure<Dimensionless> rollerVelocity) {
    this.m_rollerMotor = intakeHardware.rollerMotor;
    ROLLER_VELOCITY = rollerVelocity;
    this.m_rollerMotor = intakeHardware.rollerMotor;

    // Reset motor to defaults
    m_rollerMotor.restoreFactoryDefaults();

    // Set idle mode
    m_rollerMotor.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Initialize hardware devices for intake subsystem
   *
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware intakeHardware = new Hardware(
      new Spark(Constants.IntakeHardware.ROLLER_MOTOR_ID, MotorKind.NEO_VORTEX)
    );
    return intakeHardware;
  }

  // Tells the robot to intake
  private void intake() {
    m_rollerMotor.set(+ROLLER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
  }

  // Tells the robot to outtake
  private void outtake() {
    m_rollerMotor.set(-ROLLER_VELOCITY.in(Units.Percent), ControlType.kDutyCycle);
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
   * Stops the intake
   * @return Command to stop the intake
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }
}
