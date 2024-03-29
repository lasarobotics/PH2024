// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.lasarobotics.drive.MAXSwerveModule;
import org.lasarobotics.drive.MAXSwerveModule.ModuleLocation;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.kauailabs.NavX2InputsAutoLogged;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkInputsAutoLogged;
import org.lasarobotics.utils.GlobalConstants;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class AntiTipCommandTest {
  private final double DELTA = 5e-3;
  private DriveSubsystem m_driveSubsystem;
  private DriveSubsystem.Hardware m_drivetrainHardware;
  private Command m_antiTipCommand;

  private NavX2 m_navx;

  private Spark m_lFrontDriveMotor, m_lFrontRotateMotor;
  private Spark m_rFrontDriveMotor, m_rFrontRotateMotor;
  private Spark m_lRearDriveMotor, m_lRearRotateMotor;
  private Spark m_rRearDriveMotor, m_rRearRotateMotor;

  @BeforeEach
  public void setup() {
    HAL.initialize(500, 0);

    // Set alliance station
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

    // Create mock hardware devices
    m_navx = mock(NavX2.class);
    m_lFrontDriveMotor = mock(Spark.class);
    m_lFrontRotateMotor = mock(Spark.class);
    m_rFrontDriveMotor = mock(Spark.class);
    m_rFrontRotateMotor = mock(Spark.class);
    m_lRearDriveMotor = mock(Spark.class);
    m_lRearRotateMotor = mock(Spark.class);
    m_rRearDriveMotor = mock(Spark.class);
    m_rRearRotateMotor = mock(Spark.class);


    NavX2InputsAutoLogged navxInputs = new NavX2InputsAutoLogged();
    when(m_navx.getInputs()).thenReturn(navxInputs);

    SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();
    when(m_lFrontDriveMotor.getInputs()).thenReturn(sparkInputs);
    when(m_lFrontRotateMotor.getInputs()).thenReturn(sparkInputs);
    when(m_rFrontDriveMotor.getInputs()).thenReturn(sparkInputs);
    when(m_rFrontRotateMotor.getInputs()).thenReturn(sparkInputs);
    when(m_lRearDriveMotor.getInputs()).thenReturn(sparkInputs);
    when(m_lRearRotateMotor.getInputs()).thenReturn(sparkInputs);
    when(m_rRearDriveMotor.getInputs()).thenReturn(sparkInputs);
    when(m_rRearRotateMotor.getInputs()).thenReturn(sparkInputs);

    when(m_lFrontDriveMotor.getKind()).thenReturn(MotorKind.NEO);
    when(m_rFrontDriveMotor.getKind()).thenReturn(MotorKind.NEO);
    when(m_lRearDriveMotor.getKind()).thenReturn(MotorKind.NEO);
    when(m_rRearDriveMotor.getKind()).thenReturn(MotorKind.NEO);

    when(m_lFrontDriveMotor.getID()).thenReturn(Constants.DriveHardware.LEFT_FRONT_DRIVE_MOTOR_ID);
    when(m_lFrontRotateMotor.getID()).thenReturn(Constants.DriveHardware.LEFT_FRONT_ROTATE_MOTOR_ID);
    when(m_rFrontDriveMotor.getID()).thenReturn(Constants.DriveHardware.RIGHT_FRONT_DRIVE_MOTOR_ID);
    when(m_rFrontRotateMotor.getID()).thenReturn(Constants.DriveHardware.RIGHT_FRONT_ROTATE_MOTOR_ID);
    when(m_lRearDriveMotor.getID()).thenReturn(Constants.DriveHardware.LEFT_REAR_DRIVE_MOTOR_ID);
    when(m_lRearRotateMotor.getID()).thenReturn(Constants.DriveHardware.LEFT_REAR_ROTATE_MOTOR_ID);
    when(m_rRearDriveMotor.getID()).thenReturn(Constants.DriveHardware.RIGHT_REAR_DRIVE_MOTOR_ID);
    when(m_rRearRotateMotor.getID()).thenReturn(Constants.DriveHardware.RIGHT_REAR_ROTATE_MOTOR_ID);

    // Create hardware object using mock devices
    m_drivetrainHardware = new DriveSubsystem.Hardware(
      m_navx,
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(m_lFrontDriveMotor, m_lFrontRotateMotor),
        ModuleLocation.LeftFront,
        Constants.Drive.GEAR_RATIO,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.AUTO_LOCK_TIME,
        DriveSubsystem.MAX_SLIPPING_TIME,
        DriveSubsystem.DRIVE_CURRENT_LIMIT,
        Constants.Drive.DRIVE_SLIP_RATIO
      ),
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(m_rFrontDriveMotor, m_rFrontRotateMotor),
        ModuleLocation.RightFront,
        Constants.Drive.GEAR_RATIO,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.AUTO_LOCK_TIME,
        DriveSubsystem.MAX_SLIPPING_TIME,
        DriveSubsystem.DRIVE_CURRENT_LIMIT,
        Constants.Drive.DRIVE_SLIP_RATIO
      ),
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(m_lRearDriveMotor, m_lRearRotateMotor),
        ModuleLocation.LeftRear,
        Constants.Drive.GEAR_RATIO,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.AUTO_LOCK_TIME,
        DriveSubsystem.MAX_SLIPPING_TIME,
        DriveSubsystem.DRIVE_CURRENT_LIMIT,
        Constants.Drive.DRIVE_SLIP_RATIO
      ),
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(m_rRearDriveMotor, m_rRearRotateMotor),
        ModuleLocation.RightRear,
        Constants.Drive.GEAR_RATIO,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.AUTO_LOCK_TIME,
        DriveSubsystem.MAX_SLIPPING_TIME,
        DriveSubsystem.DRIVE_CURRENT_LIMIT,
        Constants.Drive.DRIVE_SLIP_RATIO
      )
    );

    // Create DriveSubsystem object
    m_driveSubsystem = new DriveSubsystem(
      m_drivetrainHardware,
      Constants.Drive.DRIVE_ROTATE_PID,
      Constants.Drive.DRIVE_CONTROL_CENTRICITY,
      Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
      Constants.Drive.DRIVE_TURN_INPUT_CURVE,
      Constants.Drive.DRIVE_TURN_SCALAR,
      Constants.HID.CONTROLLER_DEADBAND,
      Constants.Drive.DRIVE_LOOKAHEAD
    );

    // Create AntiTipCommand object
    m_antiTipCommand = m_driveSubsystem.ANTI_TIP_COMMAND;
  }

  @AfterEach
  public void close() {
    m_driveSubsystem.close();
    m_driveSubsystem = null;
    m_antiTipCommand = null;
  }

  /**
   * Get simulated absolute encoder input
   * @param rotation Rotation value to set
   * @param moduleLocation Swerve module location
   * @return Spark inputs to return
   */
  private SparkInputsAutoLogged getRotateSparkInput(Rotation2d rotation, MAXSwerveModule.ModuleLocation moduleLocation) {
    var sparkInputs = new SparkInputsAutoLogged();
    sparkInputs.absoluteEncoderPosition = rotation.minus(moduleLocation.offset).getRadians();

    return sparkInputs;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can execute anti-tip")
  public void execute() {
    // Hardcode sensor values
    NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
    inputs.rollAngle = Units.Degrees.of(+35.0);

    when(m_lFrontRotateMotor.getInputs()).thenReturn(getRotateSparkInput(GlobalConstants.ROTATION_PI.div(2), MAXSwerveModule.ModuleLocation.LeftFront));
    when(m_rFrontRotateMotor.getInputs()).thenReturn(getRotateSparkInput(GlobalConstants.ROTATION_PI.div(2), MAXSwerveModule.ModuleLocation.RightFront));
    when(m_lRearRotateMotor.getInputs()).thenReturn(getRotateSparkInput(GlobalConstants.ROTATION_PI.div(2), MAXSwerveModule.ModuleLocation.LeftRear));
    when(m_rRearRotateMotor.getInputs()).thenReturn(getRotateSparkInput(GlobalConstants.ROTATION_PI.div(2), MAXSwerveModule.ModuleLocation.RightRear));

    when(m_navx.getInputs()).thenReturn(inputs);

    // Try to execute anti-tip command
    m_antiTipCommand.execute();

    // Verify motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(-m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond) / 4, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond) / 4, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond) / 4, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond) / 4, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot knows when to stop anti-tip")
  public void isFinished() {
    // Hardcode sensor values
    NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
    inputs.rollAngle = Units.Degrees.of(+4.0);

    when(m_navx.getInputs()).thenReturn(inputs);

    // Check command finished condition
    boolean value = m_antiTipCommand.isFinished();

    // Assert true
    assertEquals(true, value);
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot stops and locks modules when anti-tip is complete")
  public void end() {
    // Try to end command
    m_antiTipCommand.end(false);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lFrontDriveMotor, times(1)).stopMotor();
    verify(m_lFrontRotateMotor, times(1)).stopMotor();
    verify(m_rFrontDriveMotor, times(1)).stopMotor();
    verify(m_rFrontRotateMotor, times(1)).stopMotor();
    verify(m_lRearDriveMotor, times(1)).stopMotor();
    verify(m_lRearRotateMotor, times(1)).stopMotor();
    verify(m_lRearRotateMotor, times(1)).stopMotor();
    verify(m_rRearDriveMotor, times(1)).stopMotor();
    verify(m_rRearRotateMotor, times(1)).stopMotor();
  }
}
