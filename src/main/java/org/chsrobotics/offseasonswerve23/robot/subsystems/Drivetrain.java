package org.chsrobotics.offseasonswerve23.robot.subsystems;

import javax.swing.text.Highlighter.Highlight;

import org.chsrobotics.lib.telemetry.HighLevelLogger;
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
    public void setFrontRightDriveVoltage(double volts) {
        frontRightDriveSparkMax.setVoltage(0);
    }
    public void setFrontRightSteerVoltage(double volts) {
        frontRightSteerSparkMax.setVoltage(0);
    }


    public void setFrontLefttDriveVoltage(double volts) {
        frontLeftDriveSparkMax.setVoltage(0);
    }
    public void setFrontLeftSteerVoltage(double volts) {
        frontLeftSteerSparkMax.setVoltage(0);
    }


    public void setBackRightDriveVoltage(double volts) {
        backRightDriveSparkMax.setVoltage(0);
    }
    public void setBackRightSteerVoltage(double volts) {
        backRightSteerSparkMax.setVoltage(0);
    }


    public void setBackLeftDriveVoltage(double volts) {
        backLeftDriveSparkMax.setVoltage(0);
    }
    public void setBackLeftSteerVoltage(double volts) {
        backLeftSteerSparkMax.setVoltage(0);
    }


//get velocity
    public double getFrontRightDriveVelocityMetersPerSecond() {
        return frontRightDriveSparkMax.getEncoder().getVelocity()*60*2*Math.PI*(Constants.Drivetrain.WHEEL_RADIUS_CENTIMETERS/100);
    }
    public double getFrontRightSteerVelocityRadPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(frontRightSteerSparkMax.getEncoder().getVelocity());
    }


    public double getFrontLeftDriveVelocityMetersPerSecond() {
        return frontLeftDriveSparkMax.getEncoder().getVelocity()*60*2*Math.PI*(Constants.Drivetrain.WHEEL_RADIUS_CENTIMETERS/100);
    }
    public double getFrontLeftSteerVelocityRadPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(frontLeftSteerSparkMax.getEncoder().getVelocity());
    }


    public double getBackRightDriveVelocityMetersPerSecond() {
        return backRightDriveSparkMax.getEncoder().getVelocity()*60*2*Math.PI*(Constants.Drivetrain.WHEEL_RADIUS_CENTIMETERS/100);
    }
    public double getBackRightSteerVelocityRadPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(backRightSteerSparkMax.getEncoder().getVelocity());
    }


    public double getBackLeftDriveVelocityMetersPerSecond() {
        return backLeftDriveSparkMax.getEncoder().getVelocity()*60*2*Math.PI*(Constants.Drivetrain.WHEEL_RADIUS_CENTIMETERS/100);
    }
    public double getBackLeftSteerVelocityRadPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(backLeftSteerSparkMax.getEncoder().getVelocity());
    }


    
    @Override
    public void periodic() {
        frontRightDriveVelocityMPSLogger.update(getFrontRightDriveVelocityMetersPerSecond());
        frontRightSteerVelocityRPSLogger.update(getFrontRightSteerVelocityRadPerSec());

        frontLeftDriveVelocityMPSLogger.update(getFrontLeftDriveVelocityMetersPerSecond());
        frontLeftSteerVelocityRPSLogger.update(getFrontLeftSteerVelocityRadPerSec());

        backRightDriveVelocityMPSLogger.update(getBackRightDriveVelocityMetersPerSecond());
        backRightSteerVelocityRPSLogger.update(getBackRightSteerVelocityRadPerSec());

        backLeftDriveVelocityMPSLogger.update(getBackLeftDriveVelocityMetersPerSecond());
        backLeftSteerVelocityRPSLogger.update(getBackLeftSteerVelocityRadPerSec());
    }
}
