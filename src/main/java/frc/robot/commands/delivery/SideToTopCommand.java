package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Delivery;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.Direction;

/**
 * Moves a ball from delivery into the shooter and resets its position in delivery.
 * 
 * @author Nicholas S, Michael F
 */
public class SideToTopCommand extends CommandBase {

  private final Delivery delivery;
  private Direction direction;
  private BallColor ballColor;
  private boolean isFinished;
  /** True if there is a ball there and we need to wait for it to move before checking to stop */
  private boolean waitForBallFlag;
  
  public SideToTopCommand(Delivery delivery, BallColor ballColor){
    this.delivery = delivery;
    this.ballColor = ballColor;
    addRequirements(delivery);
  }

  @Override
  public void initialize() {
    isFinished = false;
    waitForBallFlag = false;
    direction = delivery.getChamberDirection(ballColor);

    if (direction == null) {
      isFinished = true;
      return;
    }

    // Check if we need to move ball before checking to stop and start the motor
    delivery.startDelivery(direction);
    waitForBallFlag = (delivery.getTopRightSensorStatus() || delivery.getTopLeftSensorStatus());
  }

  @Override
  public void execute() {
    if (isFinished) {
      return;
    }

    // If we're waiting for old ball to move, update flag to determine its position
    if (waitForBallFlag) {
      waitForBallFlag = (delivery.getTopRightSensorStatus() || delivery.getTopLeftSensorStatus());
    } else {
      // We're finished when either of the sensors returns true
      isFinished = (delivery.getTopRightSensorStatus() || delivery.getTopLeftSensorStatus());
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Rotate internal state
    if (direction == Direction.COUNTER_CLOCKWISE) {
      delivery.rotateArrayCounterClockwise();
    } else if (direction == Direction.CLOCKWISE) {
      delivery.rotateArrayClockwise();
    }
  }

  @Override
  public boolean isFinished() {
    if (waitForBallFlag) {
      // If the old ball is still there, don't stop yet
      return false;
    } else {
      // Stop when the ball we want triggers the motors. See `end()`
      return isFinished;
    }
  }

}
