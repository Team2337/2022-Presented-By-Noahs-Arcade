package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Delivery.Direction;

/**
 * Spins the delivery until a ball is seen in the right color sensor
 */
public class SpinUntilSideSeesBall extends CommandBase {
  
  private final Delivery delivery;

  /**
   * Spins until delivery sees a ball in the right color sensor.
   * @param delivery The {@link Delivery} subsystem
   */
  public SpinUntilSideSeesBall(Delivery delivery) {
    this.delivery = delivery;
    addRequirements(delivery);
  }

  @Override
  public void initialize() {
    delivery.startDelivery(Direction.COUNTER_CLOCKWISE);
  }

  @Override
  public void end(boolean interrupted) {
    delivery.stopDelivery();
  }

  @Override
  public boolean isFinished() {
    return delivery.getRightColorSensorStatus();
  }
}
