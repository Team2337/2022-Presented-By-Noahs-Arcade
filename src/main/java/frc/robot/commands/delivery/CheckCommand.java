package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
// import frc.robot.Constants.BallColor;
import frc.robot.Constants.Directions;
import frc.robot.subsystems.Delivery;

/**
 * Moves a ball into position to be checeked and updates its color value.
 * 
 * @author Michael F, Nicholas S
 */
public class CheckCommand extends CommandBase {

  private final Delivery delivery;
  private Directions rotation;
  /** True if there is a ball there and we need to wait for it to move before checking to stop */
  private boolean checkFlag;
  
  public CheckCommand(Delivery delivery, Directions rotation) {
    this.rotation = rotation;
    this.delivery = delivery;
  }

  @Override
  public void initialize() {
    // We're moving stuff so it's no longer lined up
    delivery.linedUp = false;
    
    // Check if we need to move a ball before checking to stop and start the motor
    if (rotation == Directions.COUNTER_CLOCKWISE) {
      checkFlag = delivery.getRightColorSensorStatus();
      delivery.setDeliverySpeed(-Constants.DELIVERY_SPEED);
    } else {
      checkFlag = delivery.getLeftColorSensorStatus();
      delivery.setDeliverySpeed(Constants.DELIVERY_SPEED);
    }
  }

  @Override
  public void execute() {
    // If we're waiting for old ball to move, update flag to determine its position
    if(checkFlag){
      if (rotation == Directions.COUNTER_CLOCKWISE) {
        checkFlag = delivery.getRightColorSensorStatus();
      } else {
        checkFlag = delivery.getLeftColorSensorStatus();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Update ball values
    if (rotation == Directions.COUNTER_CLOCKWISE) {
      delivery.storedBalls[0] = delivery.storedBalls[3];
      delivery.storedBalls[2] = delivery.storedBalls[1];
      delivery.storedBalls[1] = delivery.getRightColorSensorValue(); // 1 is right
      delivery.storedBalls[3] = delivery.getLeftColorSensorValue();  // 3 is left
    } else {
      delivery.storedBalls[0] = delivery.storedBalls[1];
      delivery.storedBalls[2] = delivery.storedBalls[3];
      delivery.storedBalls[1] = delivery.getRightColorSensorValue(); // 1 is right
      delivery.storedBalls[3] = delivery.getLeftColorSensorValue();  // 3 is left
    }

    // Stop delivery
    delivery.stopDelivery();
  }

  @Override
  public boolean isFinished() {
    if (checkFlag) {
      // If we're waiting for the other ball to pass, return false
      return false;
    } else {
      // Return whether we see a ball in our wanted position or not
      if (rotation == Directions.COUNTER_CLOCKWISE) {
        return delivery.getRightColorSensorStatus();
      } else {
        return delivery.getLeftColorSensorStatus();
      }
    }
  }

}
