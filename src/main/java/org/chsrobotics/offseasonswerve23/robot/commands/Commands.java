package org.chsrobotics.offseasonswerve23.robot.commands;

import org.chsrobotics.offseasonswerve23.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Commands {
    private final Drivetrain drivetrain = new Drivetrain();

    public static CommandBase swerveInverseKinematics(Drivetrain drivetrain ) {
        return new ModuleControllers(null, null, null, null, null, null, null, null);
    }
}
