package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Delivery;

public class DeliveryOverrideCommand extends CommandBase {

  private final Delivery delivery;
  private final XboxController controller;
  
  public DeliveryOverrideCommand(Delivery delivery, XboxController controller) {
    this.delivery = delivery;
    this.controller = controller;
  }

  @Override
  public void execute() {
    if (RobotContainer.operatorStation.isBlueSwitchOn()) {
      // If blue override switch is on, turn in a counter clockwise direction according to the
      // value of the controller if it is over a certain threshold to prevent drifting.
      delivery.startDelivery(Delivery.Direction.CLOCKWISE, Math.abs(controller.getLeftX()) > 0.1 ? controller.getLeftX() * Constants.DELIVERY_SPEED : 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    delivery.stopDelivery();
  }

}
