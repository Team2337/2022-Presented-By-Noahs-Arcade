package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Delivery;

/**
 * Moves a ball into position to be shot and lines it up.
 * 
 * @author Nicholas S, Michael F
 */
public class LineupCommand extends CommandBase {

  private final Delivery delivery;

  private final double DELIVERY_SMALL_SPEED = 0.1;
  
  public LineupCommand(Delivery delivery) {
    this.delivery = delivery;
  }

  @Override
  public void initialize() {
    // Do we need to rotate?
    // TODO: are these negatives correct?
    if (delivery.getTopLeftSensorStatus() && !delivery.getTopRightSensorStatus()) {
      // Rotate so that right sensor now shows true
      delivery.setDeliverySpeed(DELIVERY_SMALL_SPEED);
    } else if (!delivery.getTopLeftSensorStatus() && delivery.getTopRightSensorStatus()) {
      // Rotate so that left sensor now shows true
      delivery.setDeliverySpeed(-DELIVERY_SMALL_SPEED);
    } 
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    delivery.linedUp = true;
    delivery.stopDelivery();
  }

  @Override
  public boolean isFinished() {
    return (delivery.getTopLeftSensorStatus() && delivery.getTopRightSensorStatus());
  }

}
