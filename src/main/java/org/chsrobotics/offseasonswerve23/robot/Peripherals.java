package org.chsrobotics.offseasonswerve23.robot;

import org.chsrobotics.lib.math.UtilityMath;
import org.chsrobotics.lib.telemetry.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Peripherals{
    private AHRS imu;
    private double currentYaw;
    private double lastYaw = 0;
    private double currentYawVelocity;

    private String subdirString = "Peripherals";


    private final Logger<Double> IMUYawLoggerRadians = new Logger<>("IMUYawRadians", subdirString);
    private final Logger<Double> IMUYawVelocityLoggerRadiansPerSecond = new Logger<>("IMUYawVelocityRadiansPerSecond", subdirString);

    private static final Peripherals peripherals = new Peripherals();

    public static Peripherals getInstance() {
        return peripherals;
    }

    private Peripherals() {
        if (TimedRobot.isReal()) {
            imu = new AHRS();
        }
    }

    public double getYawRadians() {
        if (TimedRobot.isReal()) {
            return -1*(Units.degreesToRadians(imu.getYaw()));
        } else {
            return 0;
        }
    }

    public  double getYawVelocityRadiansPerSecond() {
        return currentYawVelocity;
    }

    public void periodic() {
        currentYaw = getYawRadians();
        currentYawVelocity = UtilityMath.smallestAngleRadiansBetween(currentYaw,lastYaw)/0.02;
        lastYaw = currentYaw;

        IMUYawLoggerRadians.update(getYawRadians());
        IMUYawVelocityLoggerRadiansPerSecond.update(getYawVelocityRadiansPerSecond());
    }
}
