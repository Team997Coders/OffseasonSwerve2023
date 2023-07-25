package org.chsrobotics.offseasonswerve23.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import org.chsrobotics.lib.trajectory.motionProfile.TrapezoidProfile;
import org.chsrobotics.lib.trajectory.motionProfile.MotionProfile.State;
import org.chsrobotics.lib.trajectory.motionProfile.TrapezoidProfile.Constraints;
import org.chsrobotics.offseasonswerve23.robot.subsystems.swerve.Swerve;
import org.chsrobotics.offseasonswerve23.robot.subsystems.swerve.Swerve.SwerveModuleIO;

public class ModuleTurnProfiling extends ModuleControllers {
    public record ModuleProfiledSetpoints(DoubleSupplier driveVelocityRadiansPerSecond,
            DoubleSupplier steerPositionRadians) {
    }

    private static ModuleVelocitySetpoints transcribeDrive(ModuleProfiledSetpoints parent,
            DoubleSupplier steerVelocity) {
        return new ModuleVelocitySetpoints(steerVelocity, parent.driveVelocityRadiansPerSecond);
    }

    public ModuleTurnProfiling(Swerve swerve, ModuleProfiledSetpoints frontLeft, ModuleProfiledSetpoints frontRight,
            ModuleProfiledSetpoints backRight, ModuleProfiledSetpoints backLeft) {
        super(swerve,
                transcribeDrive(frontLeft,
                        () -> getVelocitySetpoint(swerve.getFrontLeft(), frontLeft.steerPositionRadians)),
                transcribeDrive(frontRight,
                        () -> getVelocitySetpoint(swerve.getFrontRight(), frontRight.steerPositionRadians)),
                transcribeDrive(backRight,
                        () -> getVelocitySetpoint(swerve.getBackRight(), backRight.steerPositionRadians)),
                transcribeDrive(backLeft,
                        () -> getVelocitySetpoint(swerve.getBackLeft(), backLeft.steerPositionRadians)));
    }

    private static double getVelocitySetpoint(SwerveModuleIO module, DoubleSupplier steerPositionSetpoint) {
        TrapezoidProfile profile = new TrapezoidProfile(new Constraints(20, 50),
                new State(steerPositionSetpoint.getAsDouble(), 0),
                new State(module.getSteerPositionRadians(), module.getSteerVelocityRadiansPerSecond()));

        return profile.sample(0.02).velocity;
    }
}
