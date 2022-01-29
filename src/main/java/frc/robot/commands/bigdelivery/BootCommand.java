package frc.robot.commands.bigdelivery;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Directions;
import frc.robot.subsystems.DeliveryBigBrother;

/**
 * Moves a ball of the opposing alliance's color from delivery into the shooter and resets its position in delivery.
 * 
 * <p> This is pretty much a copy of {@link ShootCommand} but for the opposing alliance color.
 * 
 * TODO: import Shooter, create Kicker
 * 
 * @author Nicholas S, Michael F
 * @implNote This could potentially be merged with {@link ShootCommand}.
 */
public class BootCommand extends CommandBase {

  private final DeliveryBigBrother bigBrother;

  private boolean ballFound = true;
  private boolean ballHasCrossed = false;

  public BootCommand(DeliveryBigBrother bigBrother){
    this.bigBrother = bigBrother;
  }

  @Override
  public void initialize() {
    // Check which way we need to rotate
    if (bigBrother.storedBalls[2] == RobotContainer.opposingColor){
      // Already there. See if we need to line it up
      if (!bigBrother.linedUp) {
        // Not lined up, do that
        new LineupCommand(bigBrother);
      }
    } else {
      // Line up
      if (bigBrother.storedBalls[3] == RobotContainer.opposingColor) {
        // Rotate from left to right
        new ChamberCommand(bigBrother, Directions.CLOCKWISE);
      } else if (bigBrother.storedBalls[1] == RobotContainer.opposingColor) {
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
    if(!ballHasCrossed) {
      ballHasCrossed = bigBrother.getShooterSensorStatus();
    }
  }

  @Override
  public void end(boolean interrupted){
    if(ballFound){
      bigBrother.storedBalls[2] = null;
      bigBrother.linedUp = false;
    } else {
      new ShootCommand(bigBrother);
    }
  }

  @Override
  public boolean isFinished() {
    return (ballHasCrossed && !bigBrother.getShooterSensorStatus()) || !ballFound;
  }

}
