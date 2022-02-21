package frc.robot.commands.delivery;

import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Delivery.Direction;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Stops the delivery motor
 * @author Madison J.
 *
 */
public class AutoStopDelivery extends CommandBase {

  private final Delivery delivery;

  public AutoStopDelivery(Delivery delivery) {
    this.delivery = delivery;

    addRequirements(delivery);
  }

  @Override
  public void execute() {
    delivery.stopDelivery();
  }

  @Override
  public void end(boolean interrupted) {
    delivery.stopDelivery();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}