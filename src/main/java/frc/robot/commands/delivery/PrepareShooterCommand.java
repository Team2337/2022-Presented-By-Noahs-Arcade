package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.Directions;
import frc.robot.subsystems.Delivery;

/**
 * Moves a ball of our alliance's color from delivery into the shooter and resets its position in delivery.
 * 
 * @author Nicholas S, Michael F
 */
public class PrepareShooterCommand extends CommandBase {

  private final Delivery delivery;
  private final BallColor ballColor;

  public PrepareShooterCommand(Delivery delivery, BallColor ballColor){
    this.delivery = delivery;
    this.ballColor = ballColor;
  }

  @Override
  public void initialize() {
    // Check which way we need to rotate
    if (delivery.storedBalls[2] == ballColor){
      // Already there. See if we need to line it up
      if (!delivery.linedUp) {
        // Not lined up, do that
        new LineupCommand(delivery);
      }
    } else {
      // Line up
      if (delivery.storedBalls[3] == ballColor) {
        // Ball is on the left, rotate clockwise
        new ChamberCommand(delivery, Directions.CLOCKWISE);
      } else if (delivery.storedBalls[1] == ballColor) {
        // Ball is on the right, rotate counter-clockwise
        new ChamberCommand(delivery, Directions.COUNTER_CLOCKWISE);
      } else if (delivery.storedBalls[0] == ballColor){
        // Ball is at the bottom, see which way to rotate without putting another ball in the lineup sensors
        if (delivery.storedBalls[1] == null){
          new ChamberCommand(delivery, Directions.COUNTER_CLOCKWISE);
        } else {
          new ChamberCommand(delivery, Directions.CLOCKWISE);
        }
      } else {
        // TODO: figure out what to do if ball *isn't* there
        DriverStation.reportError("No balls of color you want in system!", false);
      }
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted){}

  @Override
  public boolean isFinished() {
    return true;
  }

}
