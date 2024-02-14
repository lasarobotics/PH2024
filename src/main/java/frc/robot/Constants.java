// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.lasarobotics.drive.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.MAXSwerveModule;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;
import org.lasarobotics.led.LEDStrip;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.PIDConstants;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.drive.PurplePathPose;
import frc.robot.subsystems.shooter.ShooterSubsystem.State;
import frc.robot.subsystems.vision.AprilTagCamera.Resolution;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Field {
    public static final double FIELD_WIDTH = 8.21;
    public static final double FIELD_LENGTH = 16.54;

    public static final Translation2d CENTER = new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2);
    public static final Pair<Integer,Translation2d> BLUE_SPEAKER = new Pair<Integer,Translation2d>(7, new Translation2d(0.00, 5.55));
    public static final Pair<Integer,Translation2d> RED_SPEAKER = new Pair<Integer,Translation2d>(4, new Translation2d(15.64, 5.55));

    public static final PurplePathPose AMP = new PurplePathPose(
      new Pose2d(Units.Meters.of(1.85), Units.Meters.of(7.77), Rotation2d.fromDegrees(-90.0)),
      new Pose2d(Units.Meters.of(14.66), Units.Meters.of(7.77), Rotation2d.fromDegrees(-90.0)),
      Units.Meters.of(0.5),
      true
    );
    public static final PurplePathPose SOURCE = new PurplePathPose(
      new Pose2d(Units.Meters.of(15.48), Units.Meters.of(0.84), Rotation2d.fromDegrees(+120.00)),
      new Pose2d(Units.Meters.of(1.07), Units.Meters.of(0.82), Rotation2d.fromDegrees(+60.0)),
      Units.Meters.of(0.5),
      true
    );
  }

  public static class HID {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_CONTROLLER_PORT = 1;
    public static final double CONTROLLER_DEADBAND = 0.1;
  }

  public static class Drive {
    public static final PIDConstants DRIVE_ROTATE_PID = new PIDConstants(7.0, 0.0, 0.0, 0.0);
    public static final double DRIVE_SLIP_RATIO = 0.12;
    public static final double DRIVE_TURN_SCALAR = 60.0;
    public static final double DRIVE_LOOKAHEAD = 6;

    public static final ControlCentricity DRIVE_CONTROL_CENTRICITY = ControlCentricity.FIELD_CENTRIC;

    private static final double DRIVE_THROTTLE_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.000 };
    private static final double DRIVE_THROTTLE_INPUT_CURVE_Y[] = { 0.0, 0.042, 0.168, 0.378, 0.672, 1.050, 1.512, 2.508, 2.688, 3.402, 4.325 };
    private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
    private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.010, 0.050, 0.100, 0.150, 0.200, 0.250, 0.300, 0.400, 0.600, 1.0 };

    private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
    public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);

    public static final MAXSwerveModule.GearRatio GEAR_RATIO = MAXSwerveModule.GearRatio.L3;
  }

  public static class Shooter {
    public static final Measure<Distance> FLYWHEEL_DIAMETER = Units.Inches.of(2.3);
    public static final SparkPIDConfig FLYWHEEL_CONFIG = new SparkPIDConfig(
      new PIDConstants(
        0.12,
        1e-3,
        3.0,
        1 / ((Spark.MotorKind.NEO_VORTEX.getMaxRPM() / 60) * (FLYWHEEL_DIAMETER.in(Units.Meters) * Math.PI))
      ),
      false,
      true,
      0.5
    );
    public static final SparkPIDConfig ANGLE_CONFIG = new SparkPIDConfig(
      new PIDConstants(
        0.1,
        0.0,
        0.0,
        0.0
      ),
      false,
      true,
      Units.Degrees.of(0.5).in(Units.Radians),
      0.0,
      0.6,
      true
    );
    public static final FFConstants ANGLE_FF = new FFConstants(0.2, 0.02, 72.31, 0.0);
    public static final TrapezoidProfile.Constraints ANGLE_MOTION_CONSTRAINT = new TrapezoidProfile.Constraints(
      Units.DegreesPerSecond.of(180.0),
      Units.DegreesPerSecond.of(360.0).per(Units.Second)
    );
    public static final List<Entry<Measure<Distance>,State>> SHOOTER_MAP = Arrays.asList(
      Map.entry(Units.Meters.of(0.0), new State(Units.MetersPerSecond.of(15.0), Units.Degrees.of(55.0))),
      Map.entry(Units.Meters.of(1.0), new State(Units.MetersPerSecond.of(15.0), Units.Degrees.of(55.0))),
      Map.entry(Units.Meters.of(1.5), new State(Units.MetersPerSecond.of(15.0), Units.Degrees.of(50.0))),
      Map.entry(Units.Meters.of(2.0), new State(Units.MetersPerSecond.of(15.0), Units.Degrees.of(45.0))),
      Map.entry(Units.Meters.of(2.5), new State(Units.MetersPerSecond.of(15.0), Units.Degrees.of(38.0))),
      Map.entry(Units.Meters.of(3.0), new State(Units.MetersPerSecond.of(15.0), Units.Degrees.of(35.0))),
      Map.entry(Units.Meters.of(3.5), new State(Units.MetersPerSecond.of(15.0), Units.Degrees.of(32.0))),
      Map.entry(Units.Meters.of(4.0), new State(Units.MetersPerSecond.of(15.0), Units.Degrees.of(29.5))),
      Map.entry(Units.Meters.of(4.5), new State(Units.MetersPerSecond.of(15.0), Units.Degrees.of(27.5))),
      Map.entry(Units.Meters.of(5.0), new State(Units.MetersPerSecond.of(15.0), Units.Degrees.of(26.0))),
      Map.entry(Units.Meters.of(5.5), new State(Units.MetersPerSecond.of(15.0), Units.Degrees.of(25.0))),
      Map.entry(Units.Meters.of(6.0), new State(Units.MetersPerSecond.of(15.0), Units.Degrees.of(24.2)))
    );
  }

  public static class Intake {
    public static final Measure<Dimensionless> ROLLER_VELOCITY = Units.Percent.of(90);
  }

  public static class Climber {
    public static final Measure<Dimensionless> CLIMBER_VELOCITY = Units.Percent.of(80);
  }

  public static class DriveHardware {
    public static final NavX2.ID NAVX_ID = new NavX2.ID("DriveHardware/NavX2");
    public static final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Drive", 2);
    public static final Spark.ID LEFT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Rotate", 3);
    public static final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Drive", 4);
    public static final Spark.ID RIGHT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Rotate", 5);
    public static final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Drive", 6);
    public static final Spark.ID LEFT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Rotate", 7);
    public static final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Drive", 8);
    public static final Spark.ID RIGHT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Rotate", 9);
    public static final LEDStrip.ID LED_STRIP_ID = new LEDStrip.ID("DriveHardware/LEDStrip", 0, 200);
  }

  public static class IntakeHardware {
    public static final Spark.ID ROLLER_MOTOR_ID = new Spark.ID("IntakeHardware/Roller", 10);
  }

  public static class ShooterHardware {
    public static final Spark.ID TOP_FLYWHEEL_MOTOR_ID = new Spark.ID("ShooterHardware/Flywheel/Top", 11);
    public static final Spark.ID BOTTOM_FLYWHEEL_MOTOR_ID = new Spark.ID("ShooterHardware/Flywheel/Bottom", 12);
    public static final Spark.ID ANGLE_MOTOR_ID = new Spark.ID("ShooterHardware/Angle", 13);
    public static final Spark.ID INDEXER_MOTOR_ID = new Spark.ID("ShooterHardware/Indexer", 14);
  }

  public static class VisionHardware {
    public static final String CAMERA_A_NAME = "Arducam_OV9782_USB_Camera_A";
    public static final Transform3d CAMERA_A_LOCATION = new Transform3d(
      new Translation3d(0.381, 0.133, 0.102),
      new Rotation3d(0.0, Math.toRadians(-20.0), 0.0)
    );
    public static final Resolution CAMERA_A_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_A_FOV = Rotation2d.fromDegrees(79.7);

    public static final String CAMERA_B_NAME = "Arducam_OV9782_USB_Camera_B";
    public static final Transform3d CAMERA_B_LOCATION = new Transform3d(
      new Translation3d(0.148, 0.2667, 0.47),
      new Rotation3d(0.0, Math.toRadians(-25.0), Math.toRadians(+180.0))
    );
    public static final Resolution CAMERA_B_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_B_FOV = Rotation2d.fromDegrees(79.7);

    public static final String CAMERA_OBJECT_NAME = "Arducam_OV9782_USB_Camera_C";
    public static final Transform3d CAMERA_OBJECT_LOCATION = new Transform3d(
      new Translation3d(0.3, 0.0, 0.5),
      new Rotation3d(0, Math.toRadians(+15.0), 0)
    );
    public static final Resolution CAMERA_OBJECT_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_OBJECT_FOV = Rotation2d.fromDegrees(79.7);
  }

  public static class ClimberHardware {
    public static final Spark.ID LEFT_CLIMBER_MOTOR_ID = new Spark.ID("ClimberHardware/Left", 15);
    public static final Spark.ID RIGHT_CLIMBER_MOTOR_ID = new Spark.ID("ClimberHardware/Right", 16);
  }

  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
    public static final String SMARTDASHBOARD_SHOOTER_SPEED = "Shooter Speed";
    public static final String SMARTDASHBOARD_SHOOTER_ANGLE = "Shooter Angle";
  }
}