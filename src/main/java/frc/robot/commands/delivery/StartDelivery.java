package frc.robot.commands.delivery;

import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Delivery.Direction;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the kicker motor
 * @author Nicholas S
 *
 */
public class StartDelivery extends CommandBase {

  private final Delivery delivery;
  private int waitTimer;

  public StartDelivery(Delivery delivery) {
    this.delivery = delivery;

    addRequirements(delivery);
  }

  @Override
  public void initialize() {
    waitTimer = 0;
    if (delivery.getCenteringSensorStatus()) {
      delivery.stop();
    } else {
      delivery.setSpeed(Direction.COUNTER_CLOCKWISE, 0.3);
    }
  }

  @Override
  public void execute() {
    if (!delivery.getCenteringSensorStatus() || waitTimer > 5) {
      delivery.setSpeed(Direction.COUNTER_CLOCKWISE, 0.3);
    }
    waitTimer++;
  }

  @Override
  public void end(boolean interrupted) {
    delivery.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}