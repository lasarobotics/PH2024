// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.lasarobotics.battery.BatteryTracker;
import org.lasarobotics.utils.GlobalConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Robot() {
    super(GlobalConstants.ROBOT_LOOP_PERIOD);
  }

  @Override
  @SuppressWarnings("resource")
  public void robotInit() {
    // AdvantageKit Logging
    BatteryTracker batteryTracker = new BatteryTracker(BatteryTracker.initializeHardware());
    Logger.recordMetadata("ProjectName", "PurpleSwerve");
    Logger.recordMetadata("BatteryName", batteryTracker.scanBattery());

    // Set pathfinding algorithm to be AdvantageKit compatible
    Pathfinding.setPathfinder(new LocalADStarAK());

    if (isReal()) {
      // If robot is real, log to USB drive and publish data to NetworkTables
      Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution();
      // Battery Tracking
      if (batteryTracker.isBatteryReused())
        DriverStation.reportError(batteryTracker.scanBattery() + " is being reused!", false);
      else batteryTracker.writeCurrentBattery();
    } else {
      // Else just publish to NetworkTables for simulation or replay log file if var is set
      String replay = System.getenv(GlobalConstants.REPLAY_ENVIRONMENT_VAR);
      if (replay == null || replay.isBlank()) Logger.addDataReceiver(new NT4Publisher());
      else {
        // Run as fast as possible
        setUseTiming(false);
        // Pull the replay log from AdvantageScope (or prompt the user)
        String logPath = LogFileUtil.findReplayLog();
        // Read replay log
        Logger.setReplaySource(new WPILOGReader(logPath));
        // Save outputs to a new log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
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
  public void autonomousPeriodic() {}

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
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
