// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  public class Hardware {
    Spark leftMotor;
    Spark rightMotor;

    public Hardware(Spark leftMotor, Spark rightMotor) {
      this.leftMotor = leftMotor;
      this.rightMotor = rightMotor;

      this.leftMotor.follow(this.rightMotor);
    }
  }

  Spark m_leftMotor;
  Spark m_rightMotor;

  /**
   * Create an instance of ShooterSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param shooterHardware Hardware devices required by shooter
   */
  public ShooterSubsystem(Hardware shooterHardware) {
    this.m_leftMotor = shooterHardware.leftMotor;
    this.m_rightMotor = shooterHardware.rightMotor;
  }

  public Hardware initializeHardware() {
    Hardware shooterHardware = new Hardware(
      new Spark(Constants.ShooterHardware.LEFT_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.ShooterHardware.RIGHT_MOTOR_ID, MotorKind.NEO)
    );

    return shooterHardware;
  }

  private void setPower(double power) {
    m_leftMotor.set(power);
  }

  private void stop() {
    m_leftMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command shootCommand(DoubleSupplier speed) {
    return startEnd(() -> setPower(speed.getAsDouble()), () -> stop());
  }
}
