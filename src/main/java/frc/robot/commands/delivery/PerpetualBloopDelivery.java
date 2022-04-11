package frc.robot.commands.delivery;

import frc.robot.Constants;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Delivery.Direction;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the kicker motor
 * @author Nicholas S
 *
 */
public class PerpetualBloopDelivery extends CommandBase {

  private final Delivery delivery;
  private int waitTimer;
  private double deliverySpeed = 0.35;
  private Supplier<Boolean> isAtSetpoint;
  private Supplier<String> leftBallColor;

  public PerpetualBloopDelivery(Supplier<Boolean> isAtSetpoint, Supplier<String> leftBallColor, Delivery delivery) {
    this.delivery = delivery;
    this.isAtSetpoint = isAtSetpoint;
    this.leftBallColor = leftBallColor;

    addRequirements(delivery);
  }

  @Override
  public void initialize() {
    waitTimer = 0;
    if (delivery.getCenteringSensorStatus() && Constants.BallColor.getAllianceColor().toString() == leftBallColor.get()) {
      delivery.stop();
    } else {
      // if (Constants.BallColor.getAllianceColor().toString() != leftBallColor.get() && Constants.BallColor.getAllianceColor().toString() != "null" && !isAtSetpoint.get()) {
      if (!isAtSetpoint.get()) {
        delivery.stop();
      } else {
        delivery.setSpeed(Direction.CLOCKWISE, deliverySpeed);
      }
    }
  }


  @Override
  public void execute() {
    if (!delivery.getCenteringSensorStatus() || waitTimer > 5) {
      delivery.setSpeed(Direction.CLOCKWISE, deliverySpeed);
    }
    waitTimer++;
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
