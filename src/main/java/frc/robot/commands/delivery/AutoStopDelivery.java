package frc.robot.commands.delivery;

import frc.robot.subsystems.Delivery;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Stops the delivery motor
 * @author Madison J.
 *
 */
public class AutoStopDelivery extends InstantCommand {

  private final Delivery delivery;

  public AutoStopDelivery(Delivery delivery) {
    this.delivery = delivery;

    addRequirements(delivery);
  }

  @Override
  public void initialize() {
    delivery.stopDelivery();
  }

}