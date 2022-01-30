package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.Directions;
import frc.robot.subsystems.Delivery;

/**
 * Runs after we intake a ball. Rotates the delivery in the right direction by calling CheckCommand.
 * 
 * TODO: branch some of this logic out into intake commands (when those come)
 * 
 * TODO: do we need to go until intake sensor is false?
 * 
 * @author Michael F, Nicholas S
 */
public class AfterIntakeCommand extends CommandBase {

  private final Delivery delivery;
  
  public AfterIntakeCommand(Delivery delivery) {
    this.delivery = delivery;
  }

  @Override
  public void initialize() {
    // TODO: Logic can go into actual intake command.
    

    // Increase number of balls
    delivery.balls++;
    delivery.storedBalls[0] = BallColor.UNKNOWN;

    // Rotate based on how many balls we have
    if (delivery.balls == 1) {
      // 1 ball, go counter clockwise by default
      new CheckCommand(delivery, Directions.COUNTER_CLOCKWISE);

    } else if (delivery.balls == 2) {
      // 2 balls, determine which way we need to go
      if(delivery.storedBalls[3] == null){
        // Rotate this ball into right position (counter-clockwise)
        new CheckCommand(delivery, Directions.COUNTER_CLOCKWISE);
      } else {
        // Rotate this ball into left position (clockwise)
        new CheckCommand(delivery, Directions.CLOCKWISE);
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
