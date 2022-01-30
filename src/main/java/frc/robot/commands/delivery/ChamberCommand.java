package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Delivery;
import frc.robot.Constants;
import frc.robot.Constants.Directions;

/**
 * Moves a ball from delivery into the shooter and resets its position in delivery.
 * 
 * @author Nicholas S, Michael F
 */
public class ChamberCommand extends CommandBase {

  private final Delivery delivery;
  private Directions rotation;
  /** True if there is a ball there and we need to wait for it to move before checking to stop */
  private boolean checkFlag;
  
  public ChamberCommand(Delivery delivery, Directions rotation){
    this.delivery = delivery;
    this.rotation = rotation;
  }

  @Override
  public void initialize() {
    // We're moving stuff so it's no longer lined up
    delivery.linedUp = false;

    // Check if we need to move ball before checking to stop and start the motor
    if (rotation == Directions.CLOCKWISE) {
      checkFlag = delivery.getLeftColorSensorStatus();
      delivery.setDeliverySpeed(Constants.DELIVERY_SPEED);
    } else {
      checkFlag = delivery.getRightColorSensorStatus();
      delivery.setDeliverySpeed(-Constants.DELIVERY_SPEED);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    // After finished, call lineup command
    new LineupCommand(delivery);
  }

  @Override
  public boolean isFinished() {
    if (checkFlag) {
      // If the old ball is still there, don't stop yet
      return false;
    } else {
      // Stop when the ball we want triggers the motors. See `end()`
      return (delivery.getTopRightSensorStatus() || delivery.getTopLeftSensorStatus());
    }
  }

}