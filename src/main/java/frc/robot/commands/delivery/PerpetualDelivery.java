package frc.robot.commands.delivery;

import frc.robot.Constants;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Delivery.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the kicker motor
 * @author Nicholas S
 *
 */
public class PerpetualDelivery extends CommandBase {

  private final Delivery delivery;
  private int waitTimer;
  private double deliverySpeed = 0.4;

  public PerpetualDelivery(Delivery delivery) {
    this.delivery = delivery;

    addRequirements(delivery);
  }

  @Override
  public void initialize() {
    waitTimer = 0;
    if (delivery.getCenteringSensorStatus() && Constants.BallColor.getAllianceColor().toString() == DriverStation.getAlliance().toString()) {
      delivery.stop();
    } else {
      delivery.setSpeed(Direction.CLOCKWISE, deliverySpeed);
    }
  }


  @Override
  public void execute() {
    if (!delivery.getCenteringSensorStatus() || waitTimer > 5) {
      delivery.setSpeed(Direction.CLOCKWISE, deliverySpeed);
    }
    waitTimer++;

    if (delivery.getRightColorSensorStatus()) {
      delivery.stop();
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
