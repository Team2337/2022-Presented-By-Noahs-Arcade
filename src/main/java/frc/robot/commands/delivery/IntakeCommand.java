package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.Directions;
import frc.robot.subsystems.DeliveryBigBrother;

/**
 * Runs after we intake a ball. Rotates the delivery in the right direction by calling CheckCommand.
 * 
 * @author Michael F, Nicholas S
 */
public class IntakeCommand extends CommandBase {

  private final DeliveryBigBrother bigBrother;
  
  public IntakeCommand(DeliveryBigBrother bigBrother) {
    this.bigBrother = bigBrother;
  }

  @Override
  public void initialize() {
    // TODO: figure out how to properly deal with balls in the intake
    if(bigBrother.storedBalls[0] != null){
      DriverStation.reportError("Ball already in intake spot!", false);
      return;
    }

    // Increase number of balls
    bigBrother.balls++;
    bigBrother.storedBalls[0] = BallColor.UNKNOWN;

    // Rotate based on how many balls we have
    if (bigBrother.balls == 1) {
      // 1 ball, go counter clockwise by default
      new CheckCommand(bigBrother, Directions.COUNTER_CLOCKWISE);

    } else if(bigBrother.balls == 2){

      // 2 balls, determine which way we need to go
      if(bigBrother.storedBalls[3] != null){
        // Rotate this ball into left position
        new CheckCommand(bigBrother, Directions.CLOCKWISE);
      } else {
        // Rotate this ball into right position
        new CheckCommand(bigBrother, Directions.COUNTER_CLOCKWISE);
      }
      
    } else {
      // We shouldn't have more than 2 balls
      DriverStation.reportError("Too many balls! Get a penalty!", false);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }

}
