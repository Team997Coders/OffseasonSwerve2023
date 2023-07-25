package org.chsrobotics.offseasonswerve23.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import org.chsrobotics.lib.controllers.feedback.PID;
import org.chsrobotics.lib.telemetry.IntrinsicLoggable;
import org.chsrobotics.lib.util.DeltaTimeUtil;
import org.chsrobotics.offseasonswerve23.robot.Constants;
import org.chsrobotics.offseasonswerve23.robot.subsystems.swerve.Swerve;
import org.chsrobotics.offseasonswerve23.robot.subsystems.swerve.Swerve.SwerveModuleIO;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ModuleControllers extends CommandBase {
    public record ModuleVelocitySetpoints(DoubleSupplier steerSetpointRadiansPerSecond,
            DoubleSupplier driveSetpointRadiansPerSecond) {
    };

    public record ModuleControllerGains(double steerKP, double steerKI, double steerKD, double steerKA, double steerKV,
            double steerKS, double driveKP, double driveKI, double driveKD, double driveKA, double driveKV,
            double driveKS) {
    }

    private final Swerve swerve;

    private final ModuleVelocitySetpoints frontLeft;
    private final ModuleVelocitySetpoints frontRight;
    private final ModuleVelocitySetpoints backRight;
    private final ModuleVelocitySetpoints backLeft;

    private final ModuleControllerUnit frontLeftControllers = new ModuleControllerUnit(
            Constants.ModuleControllers.FRONT_LEFT_GAINS);

    private final ModuleControllerUnit frontRightControllers = new ModuleControllerUnit(
            Constants.ModuleControllers.FRONT_RIGHT_GAINS);

    private final ModuleControllerUnit backRightControllers = new ModuleControllerUnit(
            Constants.ModuleControllers.BACK_RIGHT_GAINS);

    private final ModuleControllerUnit backLeftControllers = new ModuleControllerUnit(
            Constants.ModuleControllers.BACK_LEFT_GAINS);

    private final DeltaTimeUtil dtUtil;

    public ModuleControllers(Swerve swerve, ModuleVelocitySetpoints frontLeft, ModuleVelocitySetpoints frontRight,
            ModuleVelocitySetpoints backRight,
            ModuleVelocitySetpoints backLeft) {

        addRequirements(swerve);
        this.swerve = swerve;

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        this.backLeft = backLeft;

        String subdirName = "moduleControllers";

        frontLeftControllers.autoGenerateLogs("frontLeft", subdirName);
        frontRightControllers.autoGenerateLogs("frontRight", subdirName);
        backRightControllers.autoGenerateLogs("backRight", subdirName);
        backLeftControllers.autoGenerateLogs("backLeft", subdirName);

        dtUtil = new DeltaTimeUtil();
    }

    private class ModuleControllerUnit implements IntrinsicLoggable {
        private final PID steerFeedback;
        private final SimpleMotorFeedforward steerFeedforward;

        private final PID driveFeedback;
        private final SimpleMotorFeedforward driveFeedforward;

        ModuleControllerUnit(ModuleControllerGains gains) {
            steerFeedback = new PID(gains.steerKP, gains.steerKI, gains.steerKD, 0);

            steerFeedforward = new SimpleMotorFeedforward(gains.steerKS, gains.steerKV, gains.steerKA);

            driveFeedback = new PID(gains.driveKP, gains.driveKI, gains.driveKD, 0);

            driveFeedforward = new SimpleMotorFeedforward(gains.driveKS, gains.driveKV, gains.driveKA);
        }

        double steerCalculate(double setpointRadiansPerSecond, double currentVelocity, double dt) {
            return steerFeedback.calculate(setpointRadiansPerSecond, dt)
                    + steerFeedforward.calculate(currentVelocity, setpointRadiansPerSecond, dt);
        }

        double driveCalculate(double setpointRadiansPerSecond, double currentVelocity, double dt) {
            return driveFeedback.calculate(setpointRadiansPerSecond, dt)
                    + driveFeedforward.calculate(currentVelocity, setpointRadiansPerSecond, dt);
        }

        @Override
        public void autoGenerateLogs(DataLog log, String name, String subdirName, boolean publishToNT,
                boolean recordInLog) {
            steerFeedback.autoGenerateLogs(log, name + "/steerFeedback", subdirName, publishToNT, recordInLog);
            driveFeedback.autoGenerateLogs(log, name + "/driveFeedback", subdirName, publishToNT, recordInLog);
        }

        @Override
        public void updateLogs() {
            steerFeedback.updateLogs();
            driveFeedback.updateLogs();
        }
    }

    @Override
    public void execute() {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        setControlInputs(swerve.getFrontLeft(), frontLeft, frontLeftControllers, dt);
        setControlInputs(swerve.getFrontRight(), frontRight, frontRightControllers, dt);
        setControlInputs(swerve.getBackRight(), backRight, backRightControllers, dt);
        setControlInputs(swerve.getBackLeft(), backLeft, backLeftControllers, dt);

        frontLeftControllers.updateLogs();
        frontRightControllers.updateLogs();
        backRightControllers.updateLogs();
        backLeftControllers.updateLogs();
    }

    @Override
    public void end(boolean interrupted) {
        for (SwerveModuleIO moduleIO : new SwerveModuleIO[] { swerve.getFrontLeft(), swerve.getFrontRight(),
                swerve.getBackRight(), swerve.getBackLeft() }) {
            moduleIO.setSteerControlInput(0);
            moduleIO.setDriveControlInput(0);
        }
    }

    private void setControlInputs(SwerveModuleIO module, ModuleVelocitySetpoints setpoints,
            ModuleControllerUnit controllers, double dt) {
        double steerInput = controllers.steerCalculate(setpoints.steerSetpointRadiansPerSecond.getAsDouble(),
                module.getSteerVelocityRadiansPerSecond(), dt);

        double driveInput = controllers.driveCalculate(setpoints.driveSetpointRadiansPerSecond.getAsDouble(),
                module.getDriveVelocityRadiansPerSecond(), dt);

        module.setSteerControlInput(steerInput);
        module.setDriveControlInput(driveInput);
    }
}
