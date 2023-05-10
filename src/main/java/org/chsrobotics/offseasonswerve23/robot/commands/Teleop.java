package org.chsrobotics.offseasonswerve23.robot.commands;

import org.chsrobotics.lib.input.XboxController;
import org.chsrobotics.offseasonswerve23.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Teleop extends CommandBase {
    private Drivetrain drivetrain;
    private XboxController controller = new XboxController(0);

    public Teleop(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
    }
}
