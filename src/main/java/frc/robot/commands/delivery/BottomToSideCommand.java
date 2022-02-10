package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Delivery.Direction;

/**
 * Moves a ball into position to be checeked and updates its color value.
 * 
 * @author Michael F, Nicholas S
 */
public class BottomToSideCommand extends CommandBase {

  private final Delivery delivery;
  private Direction direction;
  private boolean isFinished;
  /** True if there is a ball there and we need to wait for it to move before checking to stop */
  private boolean waitForBallFlag;
  
  public BottomToSideCommand(Delivery delivery) {
    this.delivery = delivery;
    addRequirements(delivery);
  }

  @Override
  public void initialize() {
    isFinished = false;
    waitForBallFlag = false;
    direction = delivery.getCheckRotation();

    if (direction == null) {
      isFinished = true;
      return;
    }

    // Check if we need to move a ball before checking to stop and start the motor
    if (direction == Direction.COUNTER_CLOCKWISE) {
      waitForBallFlag = delivery.getRightColorSensorStatus();
    } else if (direction == Direction.CLOCKWISE) {
      waitForBallFlag = delivery.getLeftColorSensorStatus();
    }
    delivery.startDelivery(direction);
  }

  @Override
  public void execute() {
    if (isFinished) {
      return;
    }

    // If we're waiting for old ball to move, update flag to determine its position
    if (waitForBallFlag) {
      if (direction == Direction.COUNTER_CLOCKWISE) {
        waitForBallFlag = delivery.getRightColorSensorStatus();
      } else if (direction == Direction.CLOCKWISE) {
        waitForBallFlag = delivery.getLeftColorSensorStatus();
      }
    } else {
      // Return whether we see a ball in our wanted position or not
      if (direction == Direction.COUNTER_CLOCKWISE) {
        isFinished = delivery.getRightColorSensorStatus();
      } else if (direction == Direction.CLOCKWISE) {
        isFinished = delivery.getLeftColorSensorStatus();
      } else {
        isFinished = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Update ball values
    if (direction == Direction.COUNTER_CLOCKWISE) {
      delivery.rotateArrayCounterClockwise();
    } else if (direction == Direction.CLOCKWISE) {
      delivery.rotateArrayClockwise();
    }

    // Stop delivery
    delivery.stopDelivery();
  }

  @Override
  public boolean isFinished() {
    if (waitForBallFlag) {
      // If we're waiting for the other ball to pass, return false
      return false;
    } else {
      return isFinished;
    }
  }

}
