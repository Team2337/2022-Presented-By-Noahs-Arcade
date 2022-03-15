package frc.robot.commands.delivery;

import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Delivery.Direction;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the kicker motor
 * @author Nicholas S
 *
 */
public class StartDeliveryWithLineupSensor extends CommandBase {

  private final Delivery delivery;

  public StartDeliveryWithLineupSensor(Delivery delivery) {
    this.delivery = delivery;
    addRequirements(delivery);
  }

  @Override
  public void execute() {
    if (!delivery.getCenteringSensorStatus()) {
      delivery.setSpeed(Direction.CLOCKWISE, 0.3);
    } 
  }

  @Override
  public void end(boolean interrupted) {
    delivery.stop();
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}