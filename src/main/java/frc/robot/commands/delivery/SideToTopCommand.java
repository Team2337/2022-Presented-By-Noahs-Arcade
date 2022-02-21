package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallColor;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Delivery.Direction;

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
    direction = delivery.getSideToTopDirection(ballColor);

    if (direction == null) {
      isFinished = true;
      return;
    }

    // Check if we need to move ball before checking to stop and start the motor
    delivery.start(direction);
    waitForBallFlag = delivery.getLineupSensorValue() < 2.0; //TODO: is there an actual value we can use here? I asked and nobody gave me a value
  }

  @Override
  public void execute() {
    if (isFinished) {
      return;
    }

    // If we're waiting for old ball to move, update flag to determine its position
    if (waitForBallFlag) {
      waitForBallFlag = delivery.getLineupSensorValue() < 2.0;
    } else {
      // We're finished when either of the sensors returns true
      isFinished = delivery.getLineupSensorValue() < 2.0;
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
