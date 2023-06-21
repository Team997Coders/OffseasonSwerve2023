package org.chsrobotics.offseasonswerve23.robot.commands;

import java.util.function.DoubleSupplier;

import org.chsrobotics.offseasonswerve23.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class InverseKinematics extends CommandBase {
    SwerveDriveKinematics swerveDriveKinematics;
    private final DoubleSupplier frontRightSteerSupplier;
    private final DoubleSupplier frontRightDriveSupplier;
    private final DoubleSupplier frontLeftSteerSupplier;
    private final DoubleSupplier frontLeftDriveSupplier;
    private final DoubleSupplier backRightSteerSupplier;
    private final DoubleSupplier backRightDriveSupplier;
    private final DoubleSupplier backLeftSteerSupplier;
    private final DoubleSupplier backLeftDriveSupplier;
    public InverseKinematics(ChassisSpeeds chassisSpeeds){
        this.swerveDriveKinematics =  new SwerveDriveKinematics(Constants.InverseKinematics.WHEELS_DISTANCE_FROM_CENTER);
        SwerveModuleState moduleStates[] = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        frontRightSteerSupplier. = moduleStates[0].speedMetersPerSecond;

       
    }

    @Override
    public void execute() {
        
    }
    
}
