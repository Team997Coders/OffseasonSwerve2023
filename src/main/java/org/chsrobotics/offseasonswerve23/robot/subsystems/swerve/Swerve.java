package org.chsrobotics.offseasonswerve23.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.chsrobotics.lib.telemetry.Logger;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class Swerve implements Subsystem {
    private static final String subdirString = "swerve";

    // logger for advantagescope visualizer
    private final Logger<Double[]> swerveStateLogger = new Logger<>("state", subdirString);

    public static Swerve get(boolean isReal) {
        if (isReal)
            return new SwerveHardware();
        else
            return new SwerveSimulation();
    }

    public class SwerveModuleIO {
        private final String moduleName;

        // need to be non-final if we're setting the names at construction-time
        private Logger<Double> steerPositionLogger;
        private Logger<Double> unoffsetSteerPositionLogger;
        private Logger<Double> steerVelocityLogger;

        private Logger<Double> driveAccumulationLogger;
        private Logger<Double> driveVelocityLogger;

        private Logger<Double> steerControlInputLogger;
        private Logger<Double> driveControlInputLogger;

        private Logger<Boolean> steerBrakeMode;
        private Logger<Boolean> driveBrakeMode;

        private final DoubleSupplier steerPosition;
        private final DoubleSupplier unoffsetSteerPosition;
        private final DoubleSupplier steerVelocity;

        private final DoubleSupplier driveAccumulation;
        private final DoubleSupplier driveVelocity;

        private final DoubleConsumer setSteerControlEffort;
        private final DoubleConsumer setDriveControlEffort;

        private final DoubleSupplier getSteerControlEffort;
        private final DoubleSupplier getDriveControlEffort;

        private final BooleanConsumer setSteerBrake;
        private final BooleanSupplier getSteerBrake;

        private final BooleanConsumer setDriveBrake;
        private final BooleanSupplier getDriveBrake;

        protected SwerveModuleIO(String moduleName, DoubleSupplier steerPosition, DoubleSupplier unoffsetSteer,
                DoubleSupplier steerVelocity,
                DoubleSupplier driveAccumulation, DoubleSupplier driveVelocity, DoubleConsumer setSteerControlInput,
                DoubleConsumer setDriveControlInput, DoubleSupplier getSteerControlInput,
                DoubleSupplier getDriveControlInput, BooleanConsumer setSteerBrakeMode,
                BooleanSupplier getSteerBrakeMode, BooleanConsumer setDriveBrakeMode,
                BooleanSupplier getDriveBrakeMode) {
            this.moduleName = moduleName;

            this.steerPosition = steerPosition;
            this.unoffsetSteerPosition = unoffsetSteer;
            this.steerVelocity = steerVelocity;

            this.driveAccumulation = driveAccumulation;
            this.driveVelocity = driveVelocity;

            this.setSteerControlEffort = setSteerControlInput;
            this.setDriveControlEffort = setDriveControlInput;

            this.getSteerControlEffort = getSteerControlInput;
            this.getDriveControlEffort = getDriveControlInput;

            this.setSteerBrake = setSteerBrakeMode;
            this.getSteerBrake = getSteerBrakeMode;

            this.setDriveBrake = setDriveBrakeMode;
            this.getDriveBrake = getDriveBrakeMode;

            constructLoggers();
        }

        private void constructLoggers() {
            String moduleNameString = subdirString + "/" + moduleName;

            steerPositionLogger = new Logger<>("steerPosition_radians", moduleNameString);
            unoffsetSteerPositionLogger = new Logger<>("unoffsetSteerPosition_radians", moduleNameString);
            steerVelocityLogger = new Logger<>("steerVelocity_radians_p_s", moduleNameString);

            driveAccumulationLogger = new Logger<>("driveAccumulation_radians", moduleNameString);
            driveVelocityLogger = new Logger<>("driveVelocity_radians_p_s", moduleNameString);

            steerControlInputLogger = new Logger<>("steerControlInput_volts", moduleNameString);
            driveControlInputLogger = new Logger<>("driveControlInput_volts", moduleNameString);

            steerBrakeMode = new Logger<>("steerBrakeModeEnabled", moduleNameString);
            driveBrakeMode = new Logger<>("driveBrakeModeEnabled", moduleNameString);
        }

        public double getSteerPositionRadians() {
            return steerPosition.getAsDouble();
        }

        public double getUnoffsetSteerPositionRadians() {
            return unoffsetSteerPosition.getAsDouble();
        }

        public double getSteerVelocityRadiansPerSecond() {
            return steerVelocity.getAsDouble();
        }

        public double getDriveAccumulationRadians() {
            return driveAccumulation.getAsDouble();
        }

        public double getDriveVelocityRadiansPerSecond() {
            return driveVelocity.getAsDouble();
        }

        public void setSteerControlInput(double volts) {
            setSteerControlEffort.accept(volts);
        }

        public double getSteerControlInputVolts() {
            return getSteerControlEffort.getAsDouble();
        }

        public void setDriveControlInput(double volts) {
            setDriveControlEffort.accept(volts);
        }

        public double getDriveControlInputVolts() {
            return getDriveControlEffort.getAsDouble();
        }

        public void setSteerBrakeMode(boolean enabled) {
            setSteerBrake.accept(enabled);
        }

        public boolean getSteerBrakeMode() {
            return getSteerBrake.getAsBoolean();
        }

        public void setDriveBrakeMode(boolean enabled) {
            setDriveBrake.accept(enabled);
        }

        public boolean getDriveBrakeMode() {
            return getDriveBrake.getAsBoolean();
        }

        protected void updateLogs() {
            steerPositionLogger.update(getSteerPositionRadians());
            unoffsetSteerPositionLogger.update(getUnoffsetSteerPositionRadians());
            steerVelocityLogger.update(getSteerVelocityRadiansPerSecond());

            driveAccumulationLogger.update(getDriveAccumulationRadians());
            driveVelocityLogger.update(getDriveVelocityRadiansPerSecond());

            steerControlInputLogger.update(getSteerControlInputVolts());
            driveControlInputLogger.update(getDriveControlInputVolts());

            steerBrakeMode.update(getSteerBrakeMode());
            driveBrakeMode.update(getDriveBrakeMode());
        };
    }

    public abstract SwerveModuleIO getFrontLeft();

    public abstract SwerveModuleIO getFrontRight();

    public abstract SwerveModuleIO getBackRight();

    public abstract SwerveModuleIO getBackLeft();

    @Override
    public void periodic() {
        ArrayList<Double> swerveState = new ArrayList<>();

        for (SwerveModuleIO module : new SwerveModuleIO[] {getFrontLeft(), getFrontRight(), getBackRight(), getBackLeft()}) {
            module.updateLogs();

            swerveState.add(module.steerPosition.getAsDouble());
            swerveState.add(module.driveVelocity.getAsDouble());
        }

        swerveStateLogger.update(swerveState.toArray(new Double[] {}));
    }
}
