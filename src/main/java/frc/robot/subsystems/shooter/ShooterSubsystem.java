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
    private Spark lSlaveMotor;
    private Spark rMasterMotor;

    public Hardware(Spark lSlaveMotor, Spark rMasterMotor) {
      this.lSlaveMotor = lSlaveMotor;
      this.rMasterMotor = rMasterMotor;
    }
  }
  
  private Spark m_lSlaveMotor;
  private Spark m_rMasterMotor;

  /**
   * Create an instance of ShooterSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param shooterHardware Hardware devices required by shooter
   */
  public ShooterSubsystem(Hardware shooterHardware) {
    this.m_lSlaveMotor = shooterHardware.lSlaveMotor;
    this.m_rMasterMotor = shooterHardware.rMasterMotor;

    m_lSlaveMotor.follow(m_rMasterMotor);
  }

  public Hardware initializeHardware() {
    Hardware shooterHardware = new Hardware(
      new Spark(Constants.ShooterHardware.LEFT_SLAVE_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.ShooterHardware.RIGHT_MASTER_MOTOR_ID, MotorKind.NEO)
    );

    return shooterHardware;
  }

  private void shoot(double power) {
    m_rMasterMotor.set(power);
  }

  private void stop() {
    m_rMasterMotor.stopMotor();
  }

  public Command shootCommand(DoubleSupplier speed) {
    return startEnd(() -> shoot(speed.getAsDouble()), () -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
