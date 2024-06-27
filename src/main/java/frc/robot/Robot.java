// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Robot() {
    super(GlobalConstants.ROBOT_LOOP_PERIOD);
  }

  @Override
  public void robotInit() {
    PurpleManager.initialize(
      this,
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
      Path.of("/media/sda1"),
      BuildConstants.MAVEN_NAME,
      BuildConstants.GIT_SHA,
      BuildConstants.BUILD_DATE,
      true
    );
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    PurpleManager.update();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.autonomousPeriodic();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void simulationPeriodic() {
    m_robotContainer.simulationPeriodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.getTestCommand().schedule();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
