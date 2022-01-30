package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DeliveryBigBrother;

/**
 * Moves a ball into position to be shot and lines it up.
 * 
 * @author Nicholas S, Michael F
 */
public class LineupCommand extends CommandBase {

  private final DeliveryBigBrother bigBrother;

  private final double DELIVERY_SMALL_SPEED = 0.1;
  
  public LineupCommand(DeliveryBigBrother bigBrother) {
    this.bigBrother = bigBrother;
  }

  @Override
  public void initialize() {
    // Do we need to do anything?
    if(bigBrother.linedUp)
      return;

    // Do we need to rotate? TODO: negative corrections
    if (bigBrother.getTopLeftSensorStatus() && !bigBrother.getTopRightSensorStatus()) {
      // Rotate so that right sensor now shows true
      bigBrother.delivery.setDeliverySpeed(DELIVERY_SMALL_SPEED);
    } else if (!bigBrother.getTopLeftSensorStatus() && bigBrother.getTopRightSensorStatus()) {
      // Rotate so that left sensor now shows true
      bigBrother.delivery.setDeliverySpeed(-DELIVERY_SMALL_SPEED);
    } else {
      // Both sensors are already triggering, it is likely lined up already
      bigBrother.linedUp = true;
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    bigBrother.linedUp = true;
    bigBrother.delivery.stopDelivery();
  }

  @Override
  public boolean isFinished() {
    return (bigBrother.getTopLeftSensorStatus() && bigBrother.getTopRightSensorStatus()) || bigBrother.linedUp;
  }

}
