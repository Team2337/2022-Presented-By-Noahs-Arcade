package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DeliveryBigBrother;
import frc.robot.Constants;
import frc.robot.Constants.Directions;
/**
 * Moves a ball from delivery into the shooter and resets its position in delivery.
 * 
 * TODO: import Shooter
 * 
 * @author Nicholas S, Michael F
 */
public class ChamberCommand extends CommandBase {

  private final DeliveryBigBrother bigBrother;
  private Directions rotation;
  
  public ChamberCommand(DeliveryBigBrother bigBrother, Directions rotation){
    this.bigBrother = bigBrother;
    this.rotation = rotation;
  }

  @Override
  public void initialize() {
    if (rotation == Directions.CLOCKWISE) {
      // Rotate a certain speed
      bigBrother.delivery.setDeliverySpeed(Constants.DELIVERY_SPEED);
    } else {
      bigBrother.delivery.setDeliverySpeed(-Constants.DELIVERY_SPEED);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    new LineupCommand(bigBrother);
  }

  @Override
  public boolean isFinished() {
    return (bigBrother.getTopRightSensorStatus() || bigBrother.getTopLeftSensorStatus());
  }

}