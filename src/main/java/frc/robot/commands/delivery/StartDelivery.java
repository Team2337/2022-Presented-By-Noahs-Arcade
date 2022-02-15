package frc.robot.commands.delivery;

import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Delivery.Direction;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the kicker motor
 * @author Nicholas S
 *
 */
public class StartDelivery extends CommandBase {

  private final Delivery delivery;

  public StartDelivery(Delivery delivery) {
    this.delivery = delivery;

    addRequirements(delivery);
  }

  @Override
  public void execute() {
    delivery.startDelivery(Direction.CLOCKWISE, 0.3);
  }

  @Override
  public void end(boolean interrupted) {
    delivery.stopDelivery();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}