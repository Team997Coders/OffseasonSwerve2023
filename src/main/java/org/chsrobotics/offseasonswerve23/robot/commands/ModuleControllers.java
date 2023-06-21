package org.chsrobotics.offseasonswerve23.robot.commands;

import java.util.function.DoubleSupplier;

import org.chsrobotics.lib.controllers.feedback.PID;
import org.chsrobotics.offseasonswerve23.robot.Constants;
import org.chsrobotics.offseasonswerve23.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ModuleControllers extends CommandBase{
    private  Drivetrain drivetrain = new Drivetrain();

    private final DoubleSupplier frontRightSteerSupplier;
    private final DoubleSupplier frontRightDriveSupplier;
    private final DoubleSupplier frontLeftSteerSupplier;
    private final DoubleSupplier frontLeftDriveSupplier;
    private final DoubleSupplier backRightSteerSupplier;
    private final DoubleSupplier backRightDriveSupplier;
    private final DoubleSupplier backLeftSteerSupplier;
    private final DoubleSupplier backLeftDriveSupplier;
    private final PID frontRightSteerPID = new PID(Constants.ModuleControllers.steerPIDConstants, 0, true);
    private final PID frontRightDrivePID = new PID(Constants.ModuleControllers.drivePIDConstants, 0, false);
    private final PID frontLeftSteerPID = new PID(Constants.ModuleControllers.steerPIDConstants, 0, true);
    private final PID frontLeftDrivePID = new PID(Constants.ModuleControllers.drivePIDConstants, 0, false);
    private final PID backRightSteerPID = new PID(Constants.ModuleControllers.steerPIDConstants, 0, true);
    private final PID backRightDrivePID = new PID(Constants.ModuleControllers.drivePIDConstants, 0, false);
    private final PID backLeftSteerPID = new PID(Constants.ModuleControllers.steerPIDConstants, 0, true);
    private final PID backLeftDrivePID = new PID(Constants.ModuleControllers.drivePIDConstants, 0, false);

    public ModuleControllers(DoubleSupplier frontRightSteerSupplier, DoubleSupplier frontRightDriveSupplier,DoubleSupplier frontLeftSteerSupplier,DoubleSupplier frontLeftDriveSupplier,DoubleSupplier backRightSteerSupplier,DoubleSupplier backRightDriveSupplier,DoubleSupplier backLeftSteerSupplier,DoubleSupplier backLeftDriveSupplier) {
        this.frontRightSteerSupplier = frontRightSteerSupplier;
        this.frontRightDriveSupplier = frontRightDriveSupplier;
        this.frontLeftSteerSupplier = frontLeftSteerSupplier;
        this.frontLeftDriveSupplier = frontLeftDriveSupplier;
        this.backRightSteerSupplier = backRightSteerSupplier;
        this.backRightDriveSupplier = backRightDriveSupplier;
        this.backLeftSteerSupplier = backLeftSteerSupplier;
        this.backLeftDriveSupplier = backLeftDriveSupplier;
        
    }

    @Override
    public void execute() {
        frontRightDrivePID.setSetpoint(frontRightDriveSupplier.getAsDouble());
        frontRightSteerPID.setSetpoint(frontRightSteerSupplier.getAsDouble());

        frontLeftDrivePID.setSetpoint(frontLeftDriveSupplier.getAsDouble());
        frontLeftSteerPID.setSetpoint(frontLeftSteerSupplier.getAsDouble());

        backRightDrivePID.setSetpoint(backRightDriveSupplier.getAsDouble());
        backRightSteerPID.setSetpoint(backRightSteerSupplier.getAsDouble());

        backLeftDrivePID.setSetpoint(backLeftDriveSupplier.getAsDouble());
        backLeftSteerPID.setSetpoint(backLeftSteerSupplier.getAsDouble());

        drivetrain.setFrontRightDriveVoltage(frontRightDrivePID.calculate(drivetrain.getFrontRightDriveVelocityMetersPerSec()));
        drivetrain.setFrontRightSteerVoltage(frontRightSteerPID.calculate(drivetrain.getFrontRightSteerAngleRadians()));

        drivetrain.setFrontLeftDriveVoltage(frontLeftDrivePID.calculate(drivetrain.getFrontLeftDriveVelocityMetersPerSec()));
        drivetrain.setFrontLeftSteerVoltage(frontLeftSteerPID.calculate(drivetrain.getFrontLeftSteerAngleRadians()));

        drivetrain.setBackRightDriveVoltage(backRightDrivePID.calculate(drivetrain.getBackRightDriveVelocityMetersPerSec()));
        drivetrain.setBackRightSteerVoltage(backRightSteerPID.calculate(drivetrain.getBackRightSteerAngleRadians()));

        drivetrain.setBackLeftDriveVoltage(backLeftDrivePID.calculate(drivetrain.getBackLeftDriveVelocityMetersPerSec()));
        drivetrain.setBackLeftSteerVoltage(backLeftSteerPID.calculate(drivetrain.getBackLeftSteerAngleRadians()));

        

    }

    
    
}
