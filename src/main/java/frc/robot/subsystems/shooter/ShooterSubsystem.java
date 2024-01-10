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

  Hardware h;

  /** Creates a new Shooter. */
  public ShooterSubsystem(Hardware intakHardware) {
    this.h = intakHardware;
  }

  public Hardware initializeHardware() {
    Hardware intakeHardware = new Hardware(
      new Spark(Constants.ShooterHardware.LEFT_MOTOR_ID, MotorKind.NEO), 
      new Spark(Constants.ShooterHardware.RIGHT_MOTOR_ID, MotorKind.NEO)
    );

    return intakeHardware;
  }

  private void set_power(double power) {
    h.rightMotor.set(power);
  }

  private void stop() {
    h.rightMotor.stopMotor();
  }

  public Command shootCommand(DoubleSupplier speed) {
    return startEnd(() -> set_power(speed.getAsDouble()), () -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
