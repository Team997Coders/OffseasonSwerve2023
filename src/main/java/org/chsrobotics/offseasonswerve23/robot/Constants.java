// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.chsrobotics.offseasonswerve23.robot;

import org.chsrobotics.lib.trajectory.motionProfile.TrapezoidProfile.Constraints;
import org.chsrobotics.lib.util.GearRatioHelper;
import org.chsrobotics.offseasonswerve23.robot.commands.swerve.ModuleControllers.ModuleControllerGains;
import org.chsrobotics.offseasonswerve23.robot.subsystems.swerve.SwerveHardware.SwerveModuleHardwareConstants;

import edu.wpi.first.math.system.plant.DCMotor;

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
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class Global {
    public static final double MAX_CONTROL_EFFORT_VOLTS = 11;

    public static final double ROBOT_MASS_KG = 20;

    public static final double ROBOT_MOMENT_KG_M_SQUARED = 5;
  }

  public static final class Drivetrain {
    public static final double FRONT_LEFT_X_OFFSET = -0.5;
    public static final double FRONT_LEFT_Y_OFFSET = 0.5;

    public static final double FRONT_RIGHT_X_OFFSET = 0.5;
    public static final double FRONT_RIGHT_Y_OFFSET = 0.5;

    public static final double BACK_RIGHT_X_OFFSET = 0.5;
    public static final double BACK_RIGHT_Y_OFFSET = -0.5;

    public static final double BACK_LEFT_X_OFFSET = -0.5;
    public static final double BACK_LEFT_Y_OFFSET = -0.5;

    public static final GearRatioHelper DRIVE_GEAR_RATIO = new GearRatioHelper(1, 10);
    public static final GearRatioHelper STEER_GEAR_RATIO = new GearRatioHelper(1, 10);

    public static final double WHEEL_RADIUS_METERS = 0.2;

    public static final DCMotor DRIVE_MOTOR_MODEL = DCMotor.getNEO(1)
        .withReduction(DRIVE_GEAR_RATIO.toDoubleRatioOutputToInput());
    public static final DCMotor STEER_MOTOR_MODEL = DCMotor.getNEO(1)
        .withReduction(STEER_GEAR_RATIO.toDoubleRatioOutputToInput());

    public static final double DRIVE_MOMENT_KG_M_SQUARED = 0.2;
    public static final double STEER_MOMENT_KG_M_SQUARED = 0.2;

    public static final GearRatioHelper ENCODER_WHEEL_GEAR_RATIO = new GearRatioHelper(1, 1);

    public static final SwerveModuleHardwareConstants FRONT_LEFT = new SwerveModuleHardwareConstants(0, 0, false, false,
        false, 0);
    public static final SwerveModuleHardwareConstants FRONT_RIGHT = new SwerveModuleHardwareConstants(0, 0, false,
        false, false, 0);
    public static final SwerveModuleHardwareConstants BACK_RIGHT = new SwerveModuleHardwareConstants(0, 0, false, false,
        false, 0);
    public static final SwerveModuleHardwareConstants BACK_LEFT = new SwerveModuleHardwareConstants(0, 0, false, false,
        false, 0);

    public static final boolean STEER_DEFAULT_BRAKE = false;
    public static final boolean DRIVE_DEFAULT_BRAKE = false;
  }

  public static final class ModuleControllers {
    public static final ModuleControllerGains FRONT_LEFT_GAINS = new ModuleControllerGains(1, 0, 0, 0.1, 1, 0, 1, 0, 0, 0.1,
        1, 0);

    public static final ModuleControllerGains FRONT_RIGHT_GAINS = new ModuleControllerGains(1, 0, 0, 0.1, 1, 0, 1, 0, 0,
        0.1, 1, 0);

    public static final ModuleControllerGains BACK_RIGHT_GAINS = new ModuleControllerGains(1, 0, 0, 0.1, 1, 0, 1, 0, 0, 0.1,
        1, 0);

    public static final ModuleControllerGains BACK_LEFT_GAINS = new ModuleControllerGains(1, 0, 0, 0.1, 1, 0, 1, 0, 0, 0.1,
        1, 0);
  }

  public static final class ModuleTurnProfiling {
    public static final Constraints STEER_PROFILE_CONSTRAINTS = new Constraints(20, 50);
    // rad/s, rad/s^2
  }
}