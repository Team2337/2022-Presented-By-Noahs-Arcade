package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Delivery;

public class DeliveryOverrideCommand extends CommandBase {

  private final Delivery delivery;
  private final XboxController controller;
  
  public DeliveryOverrideCommand(XboxController controller, Delivery delivery) {
    this.delivery = delivery;
    this.controller = controller;
    addRequirements(delivery);
  }

  @Override
  public void execute() {
    // Doesn't run motor unless axis is pushed psat threshold to prevent drifting
    delivery.startDelivery(Delivery.Direction.CLOCKWISE, Math.abs(controller.getLeftX()) > 0.1 ? controller.getLeftX() * Constants.DELIVERY_SPEED : 0);
  }

  @Override
  public void end(boolean interrupted) {
    delivery.stopDelivery();
  }

}
