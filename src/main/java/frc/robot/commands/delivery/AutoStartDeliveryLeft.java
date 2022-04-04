package frc.robot.commands.delivery;

import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Delivery.Direction;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the delivery motor
 * @author Madison J.
 *
 */
public class AutoStartDeliveryLeft extends CommandBase {

  private final Delivery delivery;

  public AutoStartDeliveryLeft(Delivery delivery) {
    this.delivery = delivery;

    addRequirements(delivery);
  }

  @Override
  public void execute() {
    delivery.startDelivery(Direction.COUNTER_CLOCKWISE, 0.3);
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