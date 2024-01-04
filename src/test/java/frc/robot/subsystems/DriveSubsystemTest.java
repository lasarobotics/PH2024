// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import org.lasarobotics.hardware.revrobotics.SparkMax;
import org.lasarobotics.hardware.revrobotics.SparkMaxInputsAutoLogged;
import org.lasarobotics.led.LEDStrip;
import org.mockito.AdditionalMatchers;
import org.mockito.ArgumentMatchers;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class DriveSubsystemTest {
  private final double DELTA = 5e-3;
  private DriveSubsystem m_driveSubsystem;
  private DriveSubsystem.Hardware m_drivetrainHardware;

  private NavX2 m_navx;
  private SparkMax m_lFrontDriveMotor, m_lFrontRotateMotor;
  private SparkMax m_rFrontDriveMotor, m_rFrontRotateMotor;
  private SparkMax m_lRearDriveMotor, m_lRearRotateMotor;
  private SparkMax m_rRearDriveMotor, m_rRearRotateMotor;
  private LEDStrip m_ledStrip;

  @BeforeEach
  public void setup() {
    // Create mock hardware devices
    m_navx = mock(NavX2.class);
    m_lFrontDriveMotor = mock(SparkMax.class);
    m_lFrontRotateMotor = mock(SparkMax.class);
    m_rFrontDriveMotor = mock(SparkMax.class);
    m_rFrontRotateMotor = mock(SparkMax.class);
    m_lRearDriveMotor = mock(SparkMax.class);
    m_lRearRotateMotor = mock(SparkMax.class);
    m_rRearDriveMotor = mock(SparkMax.class);
    m_rRearRotateMotor = mock(SparkMax.class);
    m_ledStrip = mock(LEDStrip.class);

    NavX2InputsAutoLogged navxInputs = new NavX2InputsAutoLogged();
    when(m_navx.getInputs()).thenReturn(navxInputs);

    SparkMaxInputsAutoLogged sparkMaxInputs = new SparkMaxInputsAutoLogged();
    when(m_lFrontDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_lFrontRotateMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rFrontDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rFrontRotateMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_lRearDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_lRearRotateMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rRearDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rRearRotateMotor.getInputs()).thenReturn(sparkMaxInputs);

    // Create hardware object using mock devices
    m_drivetrainHardware = new DriveSubsystem.Hardware(
      m_navx,
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(m_lFrontDriveMotor, m_lFrontRotateMotor),
        ModuleLocation.LeftFront,
        Constants.Drive.GEAR_RATIO,
        Constants.Drive.DRIVE_SLIP_RATIO,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.AUTO_LOCK_TIME
      ),
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(m_rFrontDriveMotor, m_rFrontRotateMotor),
        ModuleLocation.RightFront,
        Constants.Drive.GEAR_RATIO,
        Constants.Drive.DRIVE_SLIP_RATIO,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.AUTO_LOCK_TIME
      ),
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(m_lRearDriveMotor, m_lRearRotateMotor),
        ModuleLocation.LeftRear,
        Constants.Drive.GEAR_RATIO,
        Constants.Drive.DRIVE_SLIP_RATIO,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.AUTO_LOCK_TIME
      ),
      new MAXSwerveModule(
        new MAXSwerveModule.Hardware(m_rRearDriveMotor, m_rRearRotateMotor),
        ModuleLocation.RightRear,
        Constants.Drive.GEAR_RATIO,
        Constants.Drive.DRIVE_SLIP_RATIO,
        DriveSubsystem.DRIVE_WHEELBASE,
        DriveSubsystem.DRIVE_TRACK_WIDTH,
        DriveSubsystem.AUTO_LOCK_TIME
      ),
      m_ledStrip
    );

    // Create DriveSubsystem object
    m_driveSubsystem = new DriveSubsystem(
      m_drivetrainHardware,
      Constants.Drive.DRIVE_TURN_PID,
      Constants.Drive.DRIVE_CONTROL_CENTRICITY,
      Constants.Drive.DRIVE_TURN_SCALAR,
      Constants.HID.CONTROLLER_DEADBAND,
      Constants.Drive.DRIVE_LOOKAHEAD,
      Constants.Drive.DRIVE_SLIP_RATIO,
      Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
      Constants.Drive.DRIVE_TURN_INPUT_CURVE
    );

    // Disable traction control for unit tests
    m_driveSubsystem.disableTractionControl();
  }

  @AfterEach
  public void close() {
    m_driveSubsystem.close();
    m_driveSubsystem = null;
  }

  @Test
  @Order(1)
  @DisplayName("Test if robot can drive forward")
  public void forward() {
    // Hardcode sensor values
    NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
    inputs.yVelocity = +m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED;

    when(m_navx.getInputs()).thenReturn(inputs);

    // Try to drive forward
    m_driveSubsystem.teleopPID(+1.0, 0.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(2)
  @DisplayName("Test if robot can drive in reverse")
  public void reverse() {
    // Hardcode sensor values
    NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
    inputs.yVelocity = -m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED;

    when(m_navx.getInputs()).thenReturn(inputs);

    // Try to drive in reverse
    m_driveSubsystem.teleopPID(-1.0, 0.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(-m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(3)
  @DisplayName("Test if robot can strafe left")
  public void strafeLeft() {
    // Hardcode sensor values
    NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
    inputs.xVelocity = +m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED;

    when(m_navx.getInputs()).thenReturn(inputs);

    // Try to strafe left
    m_driveSubsystem.teleopPID(0.0, +1.0, 0.0);

    // Verify motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(4)
  @DisplayName("Test if robot can strafe right")
  public void strafeRight() {
    // Hardcode sensor values
    NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
    inputs.xVelocity = -m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED;

    when(m_navx.getInputs()).thenReturn(inputs);

    // Try to strafe right
    m_driveSubsystem.teleopPID(0.0, -1.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(-m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(5)
  @DisplayName("Test if robot can rotate left")
  public void rotateLeft() {
    // Hardcode sensor values
    NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
    inputs.yawRate = 90.0;

    when(m_navx.getInputs()).thenReturn(inputs);

    // Try to rotate left
    m_driveSubsystem.teleopPID(0.0, 0.0, +1.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(6)
  @DisplayName("Test if robot can rotate right")
  public void rotateRight() {
    // Hardcode sensor values
    NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
    inputs.yawRate = 90.0;

    when(m_navx.getInputs()).thenReturn(inputs);

    // Try to rotate right
    m_driveSubsystem.teleopPID(0.0, 0.0, -1.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(7)
  @DisplayName("Test if robot can stop")
  public void stop() {
    // Try to stop
    m_driveSubsystem.teleopPID(0.0, 0.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(8)
  @DisplayName("Test if robot can lock swerve modules")
  public void lock() {
    // Try to lock swerve modules
    m_driveSubsystem.lock();

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(9)
  @DisplayName("Test if robot can maintain orientation")
  public void maintainOrientation() {
    // Hardcode sensor values
    NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
    inputs.yawAngle = +30.0;

    when(m_navx.getInputs()).thenReturn(inputs);

    // Try to stay still
    m_driveSubsystem.teleopPID(0.0, 0.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(10)
  @DisplayName("Test if robot can limit wheel slip")
  public void tractionControl() {
    // Hardcode sensor values
    SparkMaxInputsAutoLogged sparkMaxInputs = new SparkMaxInputsAutoLogged();
    sparkMaxInputs.encoderVelocity = +1.0;

    when(m_lFrontDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rFrontDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_lRearDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rRearDriveMotor.getInputs()).thenReturn(sparkMaxInputs);

    // Try to drive forward with traction control
    m_driveSubsystem.enableTractionControl();
    m_driveSubsystem.teleopPID(+1.0, 0.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+0.28, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+0.28, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-0.28, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+0.28, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(11)
  @DisplayName("Test if robot can disable traction control")
  public void disableTractionControl() {
    // Hardcode sensor values
    SparkMaxInputsAutoLogged sparkMaxInputs = new SparkMaxInputsAutoLogged();
    sparkMaxInputs.encoderVelocity = +1.0;

    when(m_lFrontDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rFrontDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_lRearDriveMotor.getInputs()).thenReturn(sparkMaxInputs);
    when(m_rRearDriveMotor.getInputs()).thenReturn(sparkMaxInputs);

    // Try to drive forward without traction control
    m_driveSubsystem.disableTractionControl();
    m_driveSubsystem.teleopPID(+1.0, 0.0, 0.0);

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(-Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(-m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(+m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 2, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(12)
  @DisplayName("Test if robot can aim left towards specified point")
  public void aimLeftTowardsPoint() {
    // Rotate left towards point
    m_driveSubsystem.resetPose(new Pose2d(Constants.Field.FIELD_LENGTH / 2, Constants.Field.FIELD_WIDTH / 2, Rotation2d.fromDegrees(0.0)));
    m_driveSubsystem.aimAtPoint(new Translation2d(0.0, Constants.Field.FIELD_WIDTH));

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }

  @Test
  @Order(13)
  @DisplayName("Test if robot can aim right towards specified point")
  public void aimRightTowardsPoint() {
    // Rotate right towards point
    m_driveSubsystem.resetPose(new Pose2d(Constants.Field.FIELD_LENGTH / 2, Constants.Field.FIELD_WIDTH / 2, Rotation2d.fromDegrees(0.0)));
    m_driveSubsystem.aimAtPoint(new Translation2d(0.0, 0.0));

    // Verify that motors are being driven with expected values
    verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
    verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
    verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
  }
}
