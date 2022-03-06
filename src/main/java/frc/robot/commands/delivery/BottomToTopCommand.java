package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Delivery.Direction;

/**
 * Moves a ball from delivery into the shooter, runs after ball comes in (probably only for partner showcase)
 *
 * @author Nicholas S, Michael F
 */
public class BottomToTopCommand extends CommandBase {

  private final Delivery delivery;
  private final Kicker kicker;
  private Direction direction;
  private boolean isFinished;
  /** True if there is a ball there and we need to wait for it to move before checking to stop */
  private boolean waitForBallFlag;

  public BottomToTopCommand(Delivery delivery, Kicker kicker){
    this.delivery = delivery;
    this.kicker = kicker;
    addRequirements(delivery, kicker);
  }

  @Override
  public void initialize() {
    isFinished = false;
    waitForBallFlag = false;
    direction = Direction.COUNTER_CLOCKWISE;

    if (direction == null) {
      isFinished = true;
      return;
    }

    // Check if we need to move ball before checking to stop and start the motor
    delivery.start(direction);
    kicker.setSpeed(-0.2);
    waitForBallFlag = delivery.isBallInTopSlot();
  }

  @Override
  public void execute() {
    if (isFinished) {
      return;
    }

    // If we're waiting for old ball to move, update flag to determine its position
    if (waitForBallFlag) {
      waitForBallFlag = delivery.isBallInTopSlot();
    } else {
      // We're finished when either of the sensors returns true
      isFinished = delivery.isBallInTopSlot();
    }
  }

  @Override
  public void end(boolean interrupted) {
    delivery.stop();
    kicker.stop();
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