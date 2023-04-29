package org.chsrobotics.offseasonswerve23.robot.subsystems;

import org.chsrobotics.lib.telemetry.Logger;

import org.chsrobotics.offseasonswerve23.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Drivetrain implements Subsystem{
    private final CANSparkMax frontRightDriveSparkMax = new CANSparkMax(Constants.Drivetrain.FRONT_RIGHT_DRIVE_CANID, MotorType.kBrushless);
    private final CANSparkMax frontRightSteerSparkMax = new CANSparkMax(Constants.Drivetrain.FRONT_RIGHT_STEER_CANID, MotorType.kBrushless);

    private final CANSparkMax frontLeftDriveSparkMax = new CANSparkMax(Constants.Drivetrain.FRONT_LEFT_DRIVE_CANID  , MotorType.kBrushless);
    private final CANSparkMax frontLeftSteerSparkMax = new CANSparkMax(Constants.Drivetrain.FRONT_LEFT_STEER_CANID, MotorType.kBrushless);

    private final CANSparkMax backRightDriveSparkMax = new CANSparkMax(Constants.Drivetrain.BACK_RIGHT_DRIVE_CANID, MotorType.kBrushless);
    private final CANSparkMax backRightSteerSparkMax = new CANSparkMax(Constants.Drivetrain.BACK_RIGHT_STEER_CANID, MotorType.kBrushless);

    private final CANSparkMax backLeftDriveSparkMax = new CANSparkMax(Constants.Drivetrain.BACK_LEFT_DRIVE_CANID, MotorType.kBrushless);
    private final CANSparkMax backLeftSteerSparkMax = new CANSparkMax(Constants.Drivetrain.BACK_LEFT_STEER_CANID, MotorType.kBrushless);


    private final String subdirString = "Drivetrain";


    private final Logger<Double> frontRightDriveVelocityMPSLogger = new Logger<>("frontRightDriveVelocityMPS", subdirString);
    private final Logger<Double> frontRightSteerVelocityRPSLogger = new Logger<>("frontRightSteerVelocityRPS", subdirString);

    private final Logger<Double> frontLeftDriveVelocityMPSLogger = new Logger<>("frontLeftDriveVelocityMPS", subdirString);
    private final Logger<Double> frontLeftSteerVelocityRPSLogger = new Logger<>("frontLeftSteerVelocityRPS", subdirString);

    private final Logger<Double> backRightDriveVelocityMPSLogger = new Logger<>("backRightDriveVelocityMPS", subdirString);
    private final Logger<Double> backRightSteerVelocityRPSLogger = new Logger<>("backRightSteerVelocityRPS", subdirString);

    private final Logger<Double> backLeftDriveVelocityMPSLogger = new Logger<>("backLeftDriveVelocityMPS", subdirString);
    private final Logger<Double> backLeftSteerVelocityRPSLogger = new Logger<>("backLeftSteerVelocityRPS", subdirString);

//set voltage 

    //front right get voltage
    public void setFrontRightDriveVoltage(double volts) {
        frontRightDriveSparkMax.setVoltage(volts);
    }
    public void setFrontRightSteerVoltage(double volts) {
        frontRightSteerSparkMax.setVoltage(volts);
    }

    //front left get voltage
    public void setFrontLefttDriveVoltage(double volts) {
        frontLeftDriveSparkMax.setVoltage(volts);
    }
    public void setFrontLeftSteerVoltage(double volts) {
        frontLeftSteerSparkMax.setVoltage(volts);
    }

    //back right get voltage 
    public void setBackRightDriveVoltage(double volts) {
        backRightDriveSparkMax.setVoltage(volts);
    }
    public void setBackRightSteerVoltage(double volts) {
        backRightSteerSparkMax.setVoltage(volts);
    }

    //back left get voltage
    public void setBackLeftDriveVoltage(double volts) {
        backLeftDriveSparkMax.setVoltage(volts);
    }
    public void setBackLeftSteerVoltage(double volts) {
        backLeftSteerSparkMax.setVoltage(volts);
    }


//get velocity

    //front right get velocity
    public double getFrontRightDriveVelocityMetersPerSec() {
        return Constants.Drivetrain.ENCODER_WHEEL_GEAR_RATIO.outputFromInput((frontRightDriveSparkMax.getEncoder().getVelocity()*60)*(2*Math.PI*(Constants.Drivetrain.WHEEL_RADIUS_CENTIMETERS/100)));
    }
    public double getFrontRightSteerVelocityRadiansPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(frontRightSteerSparkMax.getEncoder().getVelocity());
    }

    //front left get velocity
    public double getFrontLeftDriveVelocityMetersPerSec() {
        return Constants.Drivetrain.ENCODER_WHEEL_GEAR_RATIO.outputFromInput((frontLeftDriveSparkMax.getEncoder().getVelocity()*60)*(2*Math.PI*(Constants.Drivetrain.WHEEL_RADIUS_CENTIMETERS/100)));
    }
    public double getFrontLeftSteerVelocityRadiansPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(frontLeftSteerSparkMax.getEncoder().getVelocity());
    }

    //back right get velocity
    public double getBackRightDriveVelocityMetersPerSec() {
        return Constants.Drivetrain.ENCODER_WHEEL_GEAR_RATIO.outputFromInput((backRightDriveSparkMax.getEncoder().getVelocity()*60)*(2*Math.PI*(Constants.Drivetrain.WHEEL_RADIUS_CENTIMETERS/100)));
    }
    public double getBackRightSteerVelocityRadiansPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(backRightSteerSparkMax.getEncoder().getVelocity());
    }

    //back left get velocity
    public double getBackLeftDriveVelocityMetersPerSec() {
        return Constants.Drivetrain.ENCODER_WHEEL_GEAR_RATIO.outputFromInput((backLeftDriveSparkMax.getEncoder().getVelocity()*60)*(2*Math.PI*(Constants.Drivetrain.WHEEL_RADIUS_CENTIMETERS/100)));
        
    }
    public double getBackLeftSteerVelocityRadiansPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(backLeftSteerSparkMax.getEncoder().getVelocity());
    }


    
    @Override
    public void periodic() {
        frontRightDriveVelocityMPSLogger.update(getFrontRightDriveVelocityMetersPerSec());
        frontRightSteerVelocityRPSLogger.update(getFrontRightSteerVelocityRadiansPerSec());

        frontLeftDriveVelocityMPSLogger.update(getFrontLeftDriveVelocityMetersPerSec());
        frontLeftSteerVelocityRPSLogger.update(getFrontLeftSteerVelocityRadiansPerSec());

        backRightDriveVelocityMPSLogger.update(getBackRightDriveVelocityMetersPerSec());
        backRightSteerVelocityRPSLogger.update(getBackRightSteerVelocityRadiansPerSec());

        backLeftDriveVelocityMPSLogger.update(getBackLeftDriveVelocityMetersPerSec());
        backLeftSteerVelocityRPSLogger.update(getBackLeftSteerVelocityRadiansPerSec());
    }
}
