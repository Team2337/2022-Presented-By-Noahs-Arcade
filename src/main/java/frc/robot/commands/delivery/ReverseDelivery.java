package frc.robot.commands.delivery;

import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Delivery.Direction;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the kicker motor
 * @author Nicholas S
 *
 */
public class ReverseDelivery extends CommandBase {

  private final Delivery delivery;
  private int waitTimer;
  private double deliverySpeed = 0.5;
  private Supplier<String> centerColorSensorStatus;

  public ReverseDelivery(Supplier<String> centerColorSensorStatus, double deliverySpeed, Delivery delivery) {
    this.delivery = delivery;
    this.deliverySpeed = deliverySpeed;
    this.centerColorSensorStatus = centerColorSensorStatus;

    addRequirements(delivery);
  }

  @Override
  public void initialize() {
    waitTimer = 0;
    if (delivery.getCenteringSensorStatus()) {
      delivery.stop();
    } else {
      delivery.setSpeed(Direction.COUNTER_CLOCKWISE, deliverySpeed);
    }
  }


  @Override
  public void execute() {
    if (!delivery.getCenteringSensorStatus() || waitTimer > 5) {
      delivery.setSpeed(Direction.COUNTER_CLOCKWISE, deliverySpeed);
    }
    waitTimer++;
  }

  @Override
  public void end(boolean interrupted) {
    delivery.stop();
  }

  @Override
  public boolean isFinished() {
    return centerColorSensorStatus.get() != "null";
  }

}
