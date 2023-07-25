package org.chsrobotics.offseasonswerve23.robot.subsystems.swerve;

import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.offseasonswerve23.robot.Constants.Drivetrain;
import org.chsrobotics.offseasonswerve23.robot.Constants.Global;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;

public class SwerveHardware extends Swerve {
    public static record SwerveModuleHardwareConstants(int steerCanID, int driveCanID, boolean steerInverted,
            boolean steerEncoderInverted, boolean driveInverted, double steerOffset) {
    }

    private final SwerveModuleIO frontLeft = moduleHelper("frontLeft",
            new CANSparkMax(Drivetrain.FRONT_LEFT.steerCanID, MotorType.kBrushless),
            new CANSparkMax(Drivetrain.FRONT_LEFT.driveCanID, MotorType.kBrushless), Drivetrain.FRONT_LEFT);
    private final SwerveModuleIO frontRight = moduleHelper("frontRight",
            new CANSparkMax(Drivetrain.FRONT_RIGHT.steerCanID, MotorType.kBrushless),
            new CANSparkMax(Drivetrain.FRONT_RIGHT.driveCanID, MotorType.kBrushless), Drivetrain.FRONT_RIGHT);
    private final SwerveModuleIO backRight = moduleHelper("backRight",
            new CANSparkMax(Drivetrain.BACK_RIGHT.steerCanID, MotorType.kBrushless),
            new CANSparkMax(Drivetrain.BACK_RIGHT.driveCanID, MotorType.kBrushless), Drivetrain.FRONT_RIGHT);
    private final SwerveModuleIO backLeft = moduleHelper("backLeft",
            new CANSparkMax(Drivetrain.BACK_LEFT.steerCanID, MotorType.kBrushless),
            new CANSparkMax(Drivetrain.BACK_LEFT.driveCanID, MotorType.kBrushless), Drivetrain.BACK_LEFT);

    public SwerveHardware() {
        register();
    }

    private SwerveModuleIO moduleHelper(String name, CANSparkMax steer, CANSparkMax drive,
            SwerveModuleHardwareConstants constants) {
        AbsoluteEncoder steerEncoder = steer.getAbsoluteEncoder(Type.kDutyCycle);
        RelativeEncoder driveEncoder = drive.getEncoder();

        return new SwerveModuleIO(name,
                () -> UtilityMath.normalizeAngleRadians(Units.rotationsToRadians(steerEncoder.getPosition())
                        * ((constants.steerEncoderInverted) ? -1 : 1) + constants.steerOffset),
                () -> UtilityMath.normalizeAngleRadians(Units.rotationsToRadians(steerEncoder.getPosition())
                        * ((constants.steerEncoderInverted) ? -1 : 1)),
                () -> Units.rotationsToRadians(steerEncoder.getVelocity())
                        * ((constants.steerEncoderInverted) ? -1 : 1),
                () -> Units.rotationsToRadians(Drivetrain.DRIVE_GEAR_RATIO.outputFromInput(driveEncoder.getPosition()))
                        * ((constants.driveInverted)
                                ? -1
                                : 1),
                () -> Units
                        .rotationsPerMinuteToRadiansPerSecond(
                                Drivetrain.DRIVE_GEAR_RATIO.outputFromInput(driveEncoder.getVelocity())
                                        * ((constants.driveInverted) ? -1 : 1)),
                (double volts) -> {
                    steer.setVoltage(UtilityMath.clamp(Global.MAX_CONTROL_EFFORT_VOLTS,
                            volts * ((constants.steerInverted) ? -1 : 1)));
                }, (double volts) -> {
                    drive.setVoltage(UtilityMath.clamp(Global.MAX_CONTROL_EFFORT_VOLTS,
                            volts * ((constants.driveInverted) ? -1 : 1)));
                },
                () -> steer.getAppliedOutput() * steer.getBusVoltage(),
                () -> drive.getAppliedOutput() * drive.getBusVoltage(),
                (boolean enabled) -> {
                    steer.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
                },
                () -> (steer.getIdleMode() == IdleMode.kBrake),
                (boolean enabled) -> {
                    drive.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
                },
                () -> (drive.getIdleMode() == IdleMode.kBrake));
    }

    @Override
    public SwerveModuleIO getFrontLeft() {
        return frontLeft;
    }

    @Override
    public SwerveModuleIO getFrontRight() {
        return frontRight;
    }

    @Override
    public SwerveModuleIO getBackRight() {
        return backRight;
    }

    @Override
    public SwerveModuleIO getBackLeft() {
        return backLeft;
    }
}
