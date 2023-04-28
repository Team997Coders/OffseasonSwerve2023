package org.chsrobotics.offseasonswerve23.robot.subsystems;

import org.chsrobotics.offseasonswerve23.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

public class Drivetrain {
    private final CANSparkMax frontRightDriveSparkMax = new CANSparkMax(Constants.Drivetrain.FRONT_RIGHT_DRIVE_CANID, MotorType.kBrushless);
    private final CANSparkMax frontRightSteerSparkMax = new CANSparkMax(Constants.Drivetrain.FRONT_RIGHT_STEER_CANID, MotorType.kBrushless);

    private final CANSparkMax frontLeftDriveSparkMax = new CANSparkMax(Constants.Drivetrain.FRONT_LEFT_DRIVE_CANID  , MotorType.kBrushless);
    private final CANSparkMax frontLeftSteerSparkMax = new CANSparkMax(Constants.Drivetrain.FRONT_LEFT_STEER_CANID, MotorType.kBrushless);

    private final CANSparkMax backRightDriveSparkMax = new CANSparkMax(Constants.Drivetrain.BACK_RIGHT_DRIVE_CANID, MotorType.kBrushless);
    private final CANSparkMax backRightSteerSparkMax = new CANSparkMax(Constants.Drivetrain.BACK_RIGHT_STEER_CANID, MotorType.kBrushless);

    private final CANSparkMax backLeftDriveSparkMax = new CANSparkMax(Constants.Drivetrain.BACK_LEFT_DRIVE_CANID, MotorType.kBrushless);
    private final CANSparkMax backLeftSteerSparkMax = new CANSparkMax(Constants.Drivetrain.BACK_LEFT_STEER_CANID, MotorType.kBrushless);


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

    public double getFrontRightDriveVelocityMetersPerSecond() {
        return frontRightDriveSparkMax.getEncoder().getVelocity();
    }
    public double getFrontRightSteerVelocityRadPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(frontRightSteerSparkMax.getEncoder().getVelocity());
    }


    public double getFrontLefttDriveVelocityMetersPerSecond() {
        return frontLeftDriveSparkMax.getEncoder().getVelocity();
    }
    public double getFrontLeftSteerVelocityRadPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(frontLeftSteerSparkMax.getEncoder().getVelocity());
    }


    public double getBackRightDriveVelocityMetersPerSecond() {
        return backRightDriveSparkMax.getEncoder().getVelocity();
    }
    public double getBackRightSteerVelocityRadPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(backRightSteerSparkMax.getEncoder().getVelocity());
    }


    public double getBackLeftDriveVelocityMetersPerSecond() {
        return backLeftDriveSparkMax.getEncoder().getVelocity();
    }
    public double getBackLeftSteerVelocityRadPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(backLeftSteerSparkMax.getEncoder().getVelocity());
    }
}
