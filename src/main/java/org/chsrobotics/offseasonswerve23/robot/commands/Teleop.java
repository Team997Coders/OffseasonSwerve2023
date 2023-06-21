package org.chsrobotics.offseasonswerve23.robot.commands;

import org.chsrobotics.lib.input.XboxController;
import org.chsrobotics.offseasonswerve23.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Teleop extends CommandBase {

   private final DoubleSupplier frontRightSteerSupplier;
   private final DoubleSupplier frontRightDriveSupplier;
   private final DoubleSupplier frontLeftSteerSupplier;
   private final DoubleSupplier frontLeftDriveSupplier;
   private final DoubleSupplier backRightSteerSupplier;
   private final DoubleSupplier backRightDriveSupplier;
   private final DoubleSupplier backLeftSteerSupplier;
   private final DoubleSupplier backLeftDriveSupplier;

private XboxController controller = new XboxController(0);
private Drivetrain drivetrain;
private ModuleControllers moduleControllers = new ModuleControllers(null, null, null, null, null, null, null, null);

 public Brake(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    boolean xButton = controller.XButton().getAsBoolean();
 }

 public void brake(boolean xButton) {
    if (xButton) {
      
    }
 }

}
