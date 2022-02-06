package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Direction;
import frc.robot.subsystems.Delivery;

/**
 * Moves a ball into position to be shot and lines it up.
 * 
 * @author Nicholas S, Michael F
 */
public class LineupTopCommand extends CommandBase {

  private final Delivery delivery;

  private final double DELIVERY_SLOW_SPEED = 0.1;
  
  public LineupTopCommand(Delivery delivery) {
    this.delivery = delivery;
    addRequirements(delivery);
  }

  @Override
  public void initialize() {
    if (!delivery.getTopLeftSensorStatus() && !delivery.getTopRightSensorStatus()) {
      // Neither sensor is triggered
      return;
    }

    // Determine which way to rotate
    // TODO: are these negatives correct?
    if (delivery.getTopLeftSensorStatus() && !delivery.getTopRightSensorStatus()) {
      // Rotate so that right sensor now shows true
      delivery.startDelivery(Direction.CLOCKWISE, DELIVERY_SLOW_SPEED);
    } else if (!delivery.getTopLeftSensorStatus() && delivery.getTopRightSensorStatus()) {
      // Rotate so that left sensor now shows true
      delivery.startDelivery(Direction.COUNTER_CLOCKWISE, DELIVERY_SLOW_SPEED);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    delivery.stopDelivery();
  }

  @Override
  public boolean isFinished() {
    return delivery.isBallLinedUpToShooter() || (!delivery.getTopLeftSensorStatus() && !delivery.getTopRightSensorStatus());
  }

}
