package org.chsrobotics.offseasonswerve23.robot.subsystems.swerve;

import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.models.CoaxialSwerveModel;
import org.chsrobotics.lib.models.CoaxialSwerveModel.CoaxialSwerveInput;
import org.chsrobotics.lib.models.CoaxialSwerveModel.CoaxialSwerveModuleOffsets;
import org.chsrobotics.lib.models.CoaxialSwerveModel.CoaxialSwerveState;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.util.DeltaTimeUtil;
import org.chsrobotics.offseasonswerve23.robot.Constants.Global;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N4;

import org.chsrobotics.offseasonswerve23.robot.Constants.Drivetrain;

public class SwerveSimulation extends Swerve {
    private final Logger<Double[]> poseLogger = new Logger<>("pose", "swerveSim");
    private final Logger<Double[]> poseDerivLogger = new Logger<>("poseDerivative", "swerveSim");

    private final CoaxialSwerveModel<N4> swerveModel;

    private final Vector<N4> steerInputs = new Vector<>(Nat.N4());
    private final Vector<N4> driveInputs = new Vector<>(Nat.N4());

    private final SwerveModuleIO frontLeft = moduleHelper("frontLeft", 0);
    private final SwerveModuleIO frontRight = moduleHelper("frontRight", 1);
    private final SwerveModuleIO backRight = moduleHelper("backRight", 2);
    private final SwerveModuleIO backLeft = moduleHelper("backLeft", 3);

    private CoaxialSwerveState<N4> state = new CoaxialSwerveState<>(0, 0, 0, 0, 0, 0, new Vector<>(Nat.N4()),
            new Vector<>(Nat.N4()), new Vector<>(Nat.N4()), new Vector<>(Nat.N4()));

    private final DeltaTimeUtil dtUtil;

    public SwerveSimulation() {
        Vector<N4> xOffsets = new Vector<>(Nat.N4());
        Vector<N4> yOffsets = new Vector<>(Nat.N4());

        xOffsets.set(0, 0, Drivetrain.FRONT_LEFT_X_OFFSET);
        yOffsets.set(0, 0, Drivetrain.FRONT_LEFT_Y_OFFSET);

        xOffsets.set(1, 0, Drivetrain.FRONT_RIGHT_X_OFFSET);
        yOffsets.set(1, 0, Drivetrain.FRONT_RIGHT_Y_OFFSET);

        xOffsets.set(2, 0, Drivetrain.BACK_RIGHT_X_OFFSET);
        yOffsets.set(2, 0, Drivetrain.BACK_RIGHT_Y_OFFSET);

        xOffsets.set(3, 0, Drivetrain.BACK_LEFT_X_OFFSET);
        yOffsets.set(3, 0, Drivetrain.BACK_LEFT_Y_OFFSET);

        swerveModel = new CoaxialSwerveModel<>(Global.ROBOT_MASS_KG,
                Global.ROBOT_MOMENT_KG_M_SQUARED, Drivetrain.DRIVE_MOMENT_KG_M_SQUARED,
                Drivetrain.STEER_MOMENT_KG_M_SQUARED, Drivetrain.DRIVE_MOTOR_MODEL, Drivetrain.STEER_MOTOR_MODEL,
                Drivetrain.WHEEL_RADIUS_METERS, new CoaxialSwerveModuleOffsets<>(xOffsets, yOffsets));

        dtUtil = new DeltaTimeUtil();
    }

    private SwerveModuleIO moduleHelper(String moduleName, int moduleIndex) {
        return new SwerveModuleIO(moduleName, () -> state.steerAngle().get(0, moduleIndex),
                () -> state.steerAngle().get(0, moduleIndex), () -> state.steerAngularVelocity().get(0, moduleIndex),
                () -> state.driveAngularAccumulation().get(0, moduleIndex),
                () -> state.driveAngularVelocity().get(0, moduleIndex),
                (double volts) -> steerInputs.set(moduleIndex, 0,
                        UtilityMath.clamp(Global.MAX_CONTROL_EFFORT_VOLTS, volts)),
                (double volts) -> driveInputs.set(moduleIndex, 0,
                        UtilityMath.clamp(Global.MAX_CONTROL_EFFORT_VOLTS, volts)),
                () -> steerInputs.get(moduleIndex, 0), () -> driveInputs.get(moduleIndex, 0), (boolean enabled) -> {
                }, () -> false, (boolean enabled) -> {
                },
                () -> false);
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

    @Override
    public void periodic() {
        state = swerveModel.simulate(state, new CoaxialSwerveInput<>(driveInputs, steerInputs),
                dtUtil.getTimeSecondsSinceLastCall());

        super.periodic();

        poseLogger.update(new Double[] { state.x(), state.y(), state.angle() });

        poseDerivLogger.update(new Double[] { state.vX(), state.vY(), state.angularVelocity() });
    }
}
