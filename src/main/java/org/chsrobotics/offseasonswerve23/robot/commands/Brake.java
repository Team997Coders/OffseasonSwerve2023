package org.chsrobotics.offseasonswerve23.robot.commands;

import org.chsrobotics.lib.input.XboxController;
import org.chsrobotics.offseasonswerve23.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class Brake extends CommandBase {
private XboxController controller = new XboxController(0);
private Drivetrain drivetrain;

 public Brake(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    boolean xButton = controller.XButton().getAsBoolean();
 }

 public void brake() {
    
 }

}
