// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import static org.mockito.ArgumentMatchers.doubleThat;
// import static org.mockito.Mockito.mock;
// import static org.mockito.Mockito.times;
// import static org.mockito.Mockito.verify;
// import static org.mockito.Mockito.when;

// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.DisplayName;
// import org.junit.jupiter.api.MethodOrderer;
// import org.junit.jupiter.api.Order;
// import org.junit.jupiter.api.Test;
// import org.junit.jupiter.api.TestMethodOrder;
// import org.lasarobotics.drive.swerve.SwerveModule;
// import org.lasarobotics.drive.swerve.child.MAXSwerveModule;
// import org.lasarobotics.drive.swerve.parent.REVSwerveModule;
// import org.lasarobotics.hardware.kauailabs.NavX2;
// import org.lasarobotics.hardware.kauailabs.NavX2InputsAutoLogged;
// import org.lasarobotics.hardware.revrobotics.Spark;
// import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
// import org.lasarobotics.hardware.revrobotics.SparkInputsAutoLogged;
// import org.lasarobotics.vision.AprilTagCamera;
// import org.mockito.AdditionalMatchers;
// import org.mockito.ArgumentMatcher;
// import org.mockito.ArgumentMatchers;

// import com.revrobotics.spark.SparkBase.ControlType;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;

// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.hal.AllianceStationID;
// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj.simulation.DriverStationSim;
// import frc.robot.AbsoluteValueMatcher;
// import frc.robot.AngleMatcher;
// import frc.robot.Constants;
// import frc.robot.subsystems.drive.DriveSubsystem;

// @TestMethodOrder(MethodOrderer.OrderAnnotation.class)
// public class DriveSubsystemTest {
//   private final double DELTA = 5e-3;
//   private DriveSubsystem m_driveSubsystem;
//   private DriveSubsystem.Hardware m_drivetrainHardware;

//   private NavX2 m_navx;
//   private Spark m_lFrontDriveMotor, m_lFrontRotateMotor;
//   private Spark m_rFrontDriveMotor, m_rFrontRotateMotor;
//   private Spark m_lRearDriveMotor, m_lRearRotateMotor;
//   private Spark m_rRearDriveMotor, m_rRearRotateMotor;

//   private ArgumentMatcher<Double> m_matchMaxLinearVelocity;

//   @BeforeEach
//   public void setup() {
//     HAL.initialize(500, 0);

//     // Set alliance station
//     DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

//     // Create mock hardware devices
//     m_navx = mock(NavX2.class);
//     m_lFrontDriveMotor = mock(Spark.class);
//     m_lFrontRotateMotor = mock(Spark.class);
//     m_rFrontDriveMotor = mock(Spark.class);
//     m_rFrontRotateMotor = mock(Spark.class);
//     m_lRearDriveMotor = mock(Spark.class);
//     m_lRearRotateMotor = mock(Spark.class);
//     m_rRearDriveMotor = mock(Spark.class);
//     m_rRearRotateMotor = mock(Spark.class);

//     NavX2InputsAutoLogged navxInputs = new NavX2InputsAutoLogged();
//     when(m_navx.getInputs()).thenReturn(navxInputs);

//     SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();
//     when(m_lFrontDriveMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_lFrontRotateMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_rFrontDriveMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_rFrontRotateMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_lRearDriveMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_lRearRotateMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_rRearDriveMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_rRearRotateMotor.getInputs()).thenReturn(sparkInputs);

//     when(m_lFrontDriveMotor.getKind()).thenReturn(MotorKind.NEO_VORTEX);
//     when(m_rFrontDriveMotor.getKind()).thenReturn(MotorKind.NEO_VORTEX);
//     when(m_lRearDriveMotor.getKind()).thenReturn(MotorKind.NEO_VORTEX);
//     when(m_rRearDriveMotor.getKind()).thenReturn(MotorKind.NEO_VORTEX);

//     when(m_lFrontDriveMotor.getID()).thenReturn(Constants.DriveHardware.LEFT_FRONT_DRIVE_MOTOR_ID);
//     when(m_lFrontRotateMotor.getID()).thenReturn(Constants.DriveHardware.LEFT_FRONT_ROTATE_MOTOR_ID);
//     when(m_rFrontDriveMotor.getID()).thenReturn(Constants.DriveHardware.RIGHT_FRONT_DRIVE_MOTOR_ID);
//     when(m_rFrontRotateMotor.getID()).thenReturn(Constants.DriveHardware.RIGHT_FRONT_ROTATE_MOTOR_ID);
//     when(m_lRearDriveMotor.getID()).thenReturn(Constants.DriveHardware.LEFT_REAR_DRIVE_MOTOR_ID);
//     when(m_lRearRotateMotor.getID()).thenReturn(Constants.DriveHardware.LEFT_REAR_ROTATE_MOTOR_ID);
//     when(m_rRearDriveMotor.getID()).thenReturn(Constants.DriveHardware.RIGHT_REAR_DRIVE_MOTOR_ID);
//     when(m_rRearRotateMotor.getID()).thenReturn(Constants.DriveHardware.RIGHT_REAR_ROTATE_MOTOR_ID);

//     // Create hardware object using mock devices
//     m_drivetrainHardware = new DriveSubsystem.Hardware(
//       m_navx,
//       MAXSwerveModule.create(
//         new REVSwerveModule.Hardware(m_lFrontDriveMotor, m_lFrontRotateMotor),
//         SwerveModule.Location.LeftFront,
//         Constants.Drive.GEAR_RATIO,
//         Constants.Drive.DRIVE_WHEEL,
//         Constants.Drive.DRIVE_PID,
//         Constants.Drive.DRIVE_FF,
//         Constants.Drive.ROTATE_PID,
//         Constants.Drive.ROTATE_FF,
//         Constants.Drive.DRIVE_SLIP_RATIO,
//         DriveSubsystem.MASS,
//         DriveSubsystem.DRIVE_WHEELBASE,
//         DriveSubsystem.DRIVE_TRACK_WIDTH,
//         DriveSubsystem.AUTO_LOCK_TIME,
//         DriveSubsystem.DRIVE_CURRENT_LIMIT
//       ),
//       MAXSwerveModule.create(
//         new REVSwerveModule.Hardware(m_rFrontDriveMotor, m_rFrontRotateMotor),
//         SwerveModule.Location.RightFront,
//         Constants.Drive.GEAR_RATIO,
//         Constants.Drive.DRIVE_WHEEL,
//         Constants.Drive.DRIVE_PID,
//         Constants.Drive.DRIVE_FF,
//         Constants.Drive.ROTATE_PID,
//         Constants.Drive.ROTATE_FF,
//         Constants.Drive.DRIVE_SLIP_RATIO,
//         DriveSubsystem.MASS,
//         DriveSubsystem.DRIVE_WHEELBASE,
//         DriveSubsystem.DRIVE_TRACK_WIDTH,
//         DriveSubsystem.AUTO_LOCK_TIME,
//         DriveSubsystem.DRIVE_CURRENT_LIMIT
//       ),
//       MAXSwerveModule.create(
//         new REVSwerveModule.Hardware(m_lRearDriveMotor, m_lRearRotateMotor),
//         SwerveModule.Location.LeftRear,
//         Constants.Drive.GEAR_RATIO,
//         Constants.Drive.DRIVE_WHEEL,
//         Constants.Drive.DRIVE_PID,
//         Constants.Drive.DRIVE_FF,
//         Constants.Drive.ROTATE_PID,
//         Constants.Drive.ROTATE_FF,
//         Constants.Drive.DRIVE_SLIP_RATIO,
//         DriveSubsystem.MASS,
//         DriveSubsystem.DRIVE_WHEELBASE,
//         DriveSubsystem.DRIVE_TRACK_WIDTH,
//         DriveSubsystem.AUTO_LOCK_TIME,
//         DriveSubsystem.DRIVE_CURRENT_LIMIT
//       ),
//       MAXSwerveModule.create(
//         new REVSwerveModule.Hardware(m_rRearDriveMotor, m_rRearRotateMotor),
//         SwerveModule.Location.RightRear,
//         Constants.Drive.GEAR_RATIO,
//         Constants.Drive.DRIVE_WHEEL,
//         Constants.Drive.DRIVE_PID,
//         Constants.Drive.DRIVE_FF,
//         Constants.Drive.ROTATE_PID,
//         Constants.Drive.ROTATE_FF,
//         Constants.Drive.DRIVE_SLIP_RATIO,
//         DriveSubsystem.MASS,
//         DriveSubsystem.DRIVE_WHEELBASE,
//         DriveSubsystem.DRIVE_TRACK_WIDTH,
//         DriveSubsystem.AUTO_LOCK_TIME,
//         DriveSubsystem.DRIVE_CURRENT_LIMIT
//       ),
//       new AprilTagCamera(
//         Constants.VisionHardware.CAMERA_A_NAME,
//         Constants.VisionHardware.CAMERA_A_LOCATION,
//         Constants.VisionHardware.CAMERA_A_RESOLUTION,
//         Constants.VisionHardware.CAMERA_A_FOV,
//         AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo)
//       ),
//       new AprilTagCamera(
//         Constants.VisionHardware.CAMERA_B_NAME,
//         Constants.VisionHardware.CAMERA_B_LOCATION,
//         Constants.VisionHardware.CAMERA_B_RESOLUTION,
//         Constants.VisionHardware.CAMERA_B_FOV,
//         AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo)
//       )
//     );

//     // Create DriveSubsystem object
//     m_driveSubsystem = new DriveSubsystem(
//       m_drivetrainHardware,
//       Constants.Drive.DRIVE_ROTATE_PID,
//       Constants.Drive.DRIVE_CONTROL_CENTRICITY,
//       Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
//       Constants.Drive.DRIVE_TURN_INPUT_CURVE,
//       Constants.Drive.DRIVE_TURN_SCALAR,
//       Constants.HID.CONTROLLER_DEADBAND,
//       Constants.Drive.DRIVE_LOOKAHEAD
//     );

//     // Max linear velocity matcher
//     m_matchMaxLinearVelocity = new AbsoluteValueMatcher(m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond), DELTA);

//     // Disable traction control for unit tests
//     m_driveSubsystem.disableTractionControlCommand().initialize();
//   }

//   @AfterEach
//   public void close() {
//     m_driveSubsystem.close();
//     m_driveSubsystem = null;
//   }

//   /**
//    * Get simulated absolute encoder input
//    * @param rotation Rotation value to set
//    * @param moduleLocation Swerve module location
//    * @return Spark inputs to return
//    */
//   private SparkInputsAutoLogged getRotateSparkInput(Rotation2d rotation, SwerveModule.Location moduleLocation) {
//     var sparkInputs = new SparkInputsAutoLogged();
//     sparkInputs.absoluteEncoderPosition = rotation.minus(moduleLocation.getLockPosition()).getRadians();

//     return sparkInputs;
//   }

//   @Test
//   @Order(1)
//   @DisplayName("Test if robot can drive forward")
//   public void forward() {
//     // Hardcode sensor values
//     NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
//     inputs.velocityY = m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.mutableCopy();

//     when(m_lFrontRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kZero, SwerveModule.Location.LeftFront));
//     when(m_rFrontRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kZero, SwerveModule.Location.RightFront));
//     when(m_lRearRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kZero, SwerveModule.Location.LeftRear));
//     when(m_rRearRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kZero, SwerveModule.Location.RightRear));

//     when(m_navx.getInputs()).thenReturn(inputs);

//     // Try to drive forward
//     m_driveSubsystem.driveCommand(() -> +1.0, () -> 0.0, () -> 0.0).execute();

//     // Verify that motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI / 2, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, 0.0, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI / 2, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//   }

//   @Test
//   @Order(2)
//   @DisplayName("Test if robot can drive in reverse")
//   public void reverse() {
//     // Hardcode sensor values
//     NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
//     inputs.velocityY = m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.unaryMinus().mutableCopy();

//     when(m_lFrontRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kZero, SwerveModule.Location.LeftFront));
//     when(m_rFrontRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kZero, SwerveModule.Location.RightFront));
//     when(m_lRearRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kZero, SwerveModule.Location.LeftRear));
//     when(m_rRearRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kZero, SwerveModule.Location.RightRear));

//     when(m_navx.getInputs()).thenReturn(inputs);

//     // Try to drive in reverse
//     m_driveSubsystem.driveCommand(() -> -1.0, () -> 0.0, () -> 0.0).execute();

//     // Verify that motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI / 2, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, 0.0, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI / 2, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//   }

//   @Test
//   @Order(3)
//   @DisplayName("Test if robot can strafe left")
//   public void strafeLeft() {
//     // Hardcode sensor values
//     NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
//     inputs.velocityX = m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.mutableCopy();

//     when(m_lFrontRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kPi.div(2), SwerveModule.Location.LeftFront));
//     when(m_rFrontRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kPi.div(2), SwerveModule.Location.RightFront));
//     when(m_lRearRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kPi.div(2), SwerveModule.Location.LeftRear));
//     when(m_rRearRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kPi.div(2), SwerveModule.Location.RightRear));

//     when(m_navx.getInputs()).thenReturn(inputs);

//     // Try to strafe left
//     m_driveSubsystem.driveCommand(() -> 0.0, () -> +1.0, () -> 0.0).execute();

//     // Verify motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI / 2, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI / 2, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, 0.0, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//   }

//   @Test
//   @Order(4)
//   @DisplayName("Test if robot can strafe right")
//   public void strafeRight() {
//     // Hardcode sensor values
//     NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
//     inputs.velocityX = m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.unaryMinus().mutableCopy();

//     when(m_lFrontRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kPi.div(2), SwerveModule.Location.LeftFront));
//     when(m_rFrontRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kPi.div(2), SwerveModule.Location.RightFront));
//     when(m_lRearRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kPi.div(2), SwerveModule.Location.LeftRear));
//     when(m_rRearRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kPi.div(2), SwerveModule.Location.RightRear));

//     when(m_navx.getInputs()).thenReturn(inputs);

//     // Try to strafe right
//     m_driveSubsystem.driveCommand(() -> 0.0, () -> -1.0, () -> 0.0).execute();

//     // Verify that motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI / 2, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI / 2, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, 0.0, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//   }

//   @Test
//   @Order(5)
//   @DisplayName("Test if robot can rotate left")
//   public void rotateLeft() {
//     // Hardcode sensor values
//     NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
//     inputs.yawRate = Units.DegreesPerSecond.of(90.0).mutableCopy();

//     when(m_navx.getInputs()).thenReturn(inputs);

//     // Try to rotate left
//     m_driveSubsystem.driveCommand(() -> 0.0, () -> 0.0, () -> +1.0).execute();

//     // Verify that motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//   }

//   @Test
//   @Order(6)
//   @DisplayName("Test if robot can rotate right")
//   public void rotateRight() {
//     // Hardcode sensor values
//     NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
//     inputs.yawRate = Units.DegreesPerSecond.of(90.0).mutableCopy();

//     when(m_navx.getInputs()).thenReturn(inputs);

//     // Try to rotate right
//     m_driveSubsystem.driveCommand(() -> 0.0, () -> 0.0, () -> -1.0).execute();;

//     // Verify that motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//   }

//   @Test
//   @Order(7)
//   @DisplayName("Test if robot can stop")
//   public void stop() {
//     // Try to stop
//     m_driveSubsystem.driveCommand(() -> 0.0, () -> 0.0, () -> 0.0).execute();

//     // Verify that motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//   }

//   @Test
//   @Order(8)
//   @DisplayName("Test if robot can lock swerve modules")
//   public void lock() {
//     // Try to lock swerve modules
//     m_driveSubsystem.lockCommand().initialize();;

//     // Verify that motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.eq(0.0, DELTA), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//   }

//   @Test
//   @Order(9)
//   @DisplayName("Test if robot can maintain orientation")
//   public void maintainOrientation() {
//     // Hardcode sensor values
//     NavX2InputsAutoLogged inputs = new NavX2InputsAutoLogged();
//     inputs.yawAngle = Units.Degrees.of(+30.0).mutableCopy();

//     when(m_navx.getInputs()).thenReturn(inputs);

//     // Try to stay still
//     m_driveSubsystem.driveCommand(() -> 0.0, () -> 0.0, () -> 0.0).execute();

//     // Verify that motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//   }

//   @Test
//   @Order(10)
//   @DisplayName("Test if robot can limit wheel slip")
//   public void tractionControl() {
//     // Hardcode sensor values
//     SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();
//     sparkInputs.encoderVelocity = +4.3;

//     when(m_lFrontDriveMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_rFrontDriveMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_lRearDriveMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_rRearDriveMotor.getInputs()).thenReturn(sparkInputs);

//     // Try to drive forward with traction control
//     m_driveSubsystem.enableTractionControlCommand().initialize();
//     m_driveSubsystem.driveCommand(() -> +0.5, () -> 0.0, () -> 0.0).execute();

//     // Verify that motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond) / 2), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI / 2, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond) / 2), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, 0.0, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.lt(m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond) / 2), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, 0.0, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.lt(m_driveSubsystem.DRIVE_MAX_LINEAR_SPEED.in(Units.MetersPerSecond) / 2), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI / 2, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//   }

//   @Test
//   @Order(11)
//   @DisplayName("Test if robot can disable traction control")
//   public void disableTractionControl() {
//     // Hardcode sensor values
//     SparkInputsAutoLogged sparkInputs = new SparkInputsAutoLogged();
//     sparkInputs.encoderVelocity = +1.0;

//     when(m_lFrontDriveMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_rFrontDriveMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_lRearDriveMotor.getInputs()).thenReturn(sparkInputs);
//     when(m_rRearDriveMotor.getInputs()).thenReturn(sparkInputs);

//     when(m_lFrontRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kZero, SwerveModule.Location.LeftFront));
//     when(m_rFrontRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kZero, SwerveModule.Location.RightFront));
//     when(m_lRearRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kZero, SwerveModule.Location.LeftRear));
//     when(m_rRearRotateMotor.getInputs()).thenReturn(getRotateSparkInput(Rotation2d.kZero, SwerveModule.Location.RightRear));

//     // Try to drive forward without traction control
//     m_driveSubsystem.disableTractionControlCommand().initialize();
//     m_driveSubsystem.driveCommand(() -> +1.0, () -> 0.0, () -> 0.0).execute();

//     // Verify that motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI / 2, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, 0.0, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(doubleThat(m_matchMaxLinearVelocity), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(doubleThat(new AngleMatcher(AngleMatcher.Units.RADIANS, Math.PI / 2, DELTA)), ArgumentMatchers.eq(ControlType.kPosition));
//   }

//   @Test
//   @Order(12)
//   @DisplayName("Test if robot can aim left towards specified point")
//   public void aimLeftTowardsPoint() {
//     // Rotate left towards point
//     var initialPose = new Pose2d(Constants.Field.FIELD_LAYOUT.getFieldLength() / 2, Constants.Field.FIELD_LAYOUT.getFieldWidth() / 2, Rotation2d.fromDegrees(0.0));
//     m_driveSubsystem.resetPoseCommand(() -> initialPose).initialize();
//     m_driveSubsystem.aimAtPointCommand(new Translation2d(0.0, Constants.Field.FIELD_LAYOUT.getFieldWidth()), false, true).execute();

//     // Verify that motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.gt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//   }

//   @Test
//   @Order(13)
//   @DisplayName("Test if robot can aim right towards specified point")
//   public void aimRightTowardsPoint() {
//     // Rotate right towards point
//     var initialPose = new Pose2d(Constants.Field.FIELD_LAYOUT.getFieldLength() / 2, Constants.Field.FIELD_LAYOUT.getFieldWidth() / 2, Rotation2d.fromDegrees(0.0));
//     m_driveSubsystem.resetPoseCommand(() -> initialPose).initialize();
//     m_driveSubsystem.aimAtPointCommand(new Translation2d(0.0, 0.0), false, true).execute();

//     // Verify that motors are being driven with expected values
//     verify(m_lFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rFrontDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rFrontRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_lRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_lRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//     verify(m_rRearDriveMotor, times(1)).set(AdditionalMatchers.lt(0.0), ArgumentMatchers.eq(ControlType.kVelocity));
//     verify(m_rRearRotateMotor, times(1)).set(AdditionalMatchers.eq(+Math.PI / 4, DELTA), ArgumentMatchers.eq(ControlType.kPosition));
//   }
// }
