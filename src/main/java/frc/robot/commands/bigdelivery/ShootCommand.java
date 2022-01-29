package frc.robot.commands.bigdelivery;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Directions;
import frc.robot.subsystems.DeliveryBigBrother;

/**
 * Moves a ball of our alliance's color from delivery into the shooter and resets its position in delivery.
 * 
 * TODO: import Shooter, create Kicker
 * 
 * @author Nicholas S, Michael F
 */
public class ShootCommand extends CommandBase {

  private final DeliveryBigBrother bigBrother;

  private boolean ballFound = true;
  private boolean ballHasCrossed = false;

  public ShootCommand(DeliveryBigBrother bigBrother){
    this.bigBrother = bigBrother;
  }

  @Override
  public void initialize() {
    // Check which way we need to rotate
    if (bigBrother.storedBalls[2] == RobotContainer.allianceColor){
      // Already there. See if we need to line it up
      if (!bigBrother.linedUp) {
        // Not lined up, do that
        new LineupCommand(bigBrother);
      }
    } else {
      // Line up
      if (bigBrother.storedBalls[3] == RobotContainer.allianceColor) {
        // Rotate from left to right
        new ChamberCommand(bigBrother, Directions.CLOCKWISE);
      } else if (bigBrother.storedBalls[1] == RobotContainer.allianceColor) {
        // Rotate from right to left
        new ChamberCommand(bigBrother, Directions.COUNTER_CLOCKWISE);
      } else {
        // TODO: figure out what to do if ball *isn't* there
        DriverStation.reportError("No balls of color you want in system!", false);
        ballFound = false;
      }
    }

    // TODO: create kicker code
  }

  @Override
  public void execute() {
    if(!ballHasCrossed) { //This is literally not relevant
      ballHasCrossed = bigBrother.getShooterSensorStatus();
    }
  }

  @Override
  public void end(boolean interrupted){
    if(ballFound){
      bigBrother.storedBalls[2] = null;
      bigBrother.linedUp = false;
    }
  }

  @Override
  public boolean isFinished() {
    return (ballHasCrossed && !bigBrother.getShooterSensorStatus()) || !ballFound;
  }

}
