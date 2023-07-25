// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.chsrobotics.offseasonswerve23.robot;

import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.offseasonswerve23.robot.commands.swerve.ModuleTurnProfiling;
import org.chsrobotics.offseasonswerve23.robot.commands.swerve.ModuleTurnProfiling.ModuleProfiledSetpoints;
import org.chsrobotics.offseasonswerve23.robot.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private final Swerve swerve = Swerve.get(isReal());

  @Override
  public void robotInit() {
    HighLevelLogger.getInstance().autoGenerateLogs("System", "system");

    HighLevelLogger.getInstance().startLogging();
  }

  @Override
  public void robotPeriodic() {
    swerve.periodic();

    HighLevelLogger.getInstance().updateLogs();

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance()
        .schedule(new ModuleTurnProfiling(swerve, new ModuleProfiledSetpoints(() -> 0, () -> 1),
            new ModuleProfiledSetpoints(() -> 0, () -> 1), new ModuleProfiledSetpoints(() -> 0, () -> 1),
            new ModuleProfiledSetpoints(() -> 0, () -> 1)));
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
