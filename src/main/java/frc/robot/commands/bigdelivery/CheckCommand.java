package frc.robot.commands.bigdelivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.BallColor;
import frc.robot.Constants.Directions;
import frc.robot.subsystems.DeliveryBigBrother;

/**
 * Moves a ball into position to be checeked and updates its color value.
 * 
 * @author Michael F, Nicholas S
 */
public class CheckCommand extends CommandBase {

  private final DeliveryBigBrother bigBrother;
  private Directions rotation;
  private boolean checkFlag;
  
  public CheckCommand(DeliveryBigBrother bigBrother, Directions rotation) {
    this.rotation = rotation;
    this.bigBrother = bigBrother;
  }

  @Override
  public void initialize() {
    // See if we need to wait for a ball to move before checking color
    if (rotation == Directions.COUNTER_CLOCKWISE) {
      checkFlag = bigBrother.storedBalls[1] == null;
    } else {
      checkFlag = bigBrother.storedBalls[3] == null;
    }
  }

  @Override
  public void execute() {
    // If we're waiting for old ball to move, update flag to determine its position
    if(checkFlag){
      if (rotation == Directions.COUNTER_CLOCKWISE) {
        checkFlag = !bigBrother.getRightColorSensorStatus();
      } else {
        checkFlag = !bigBrother.getLeftColorSensorStatus();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Update ball values
    if (rotation == Directions.COUNTER_CLOCKWISE) {
      bigBrother.storedBalls[0] = bigBrother.storedBalls[3];
      bigBrother.storedBalls[2] = bigBrother.storedBalls[1];
      bigBrother.storedBalls[1] = bigBrother.getRightColorSensorValue(); // 1 is right
      bigBrother.storedBalls[3] = bigBrother.getLeftColorSensorValue();  // 3 is left
    } else {
      bigBrother.storedBalls[0] = bigBrother.storedBalls[1];
      bigBrother.storedBalls[2] = bigBrother.storedBalls[3];
      bigBrother.storedBalls[1] = bigBrother.getRightColorSensorValue(); // 1 is right
      bigBrother.storedBalls[3] = bigBrother.getLeftColorSensorValue();  // 3 is left
    }
  }

  @Override
  public boolean isFinished() {
    // If we're waiting for the other ball to pass, return false
    if(!checkFlag) return false;

    // Return whether we see a ball in our wanted position or not
    if (rotation == Directions.COUNTER_CLOCKWISE) {
      return bigBrother.getRightColorSensorStatus();
    } else {
      return bigBrother.getLeftColorSensorStatus();
    }
  }

}
