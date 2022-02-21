package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Delivery.Direction;

/**
 * Moves a ball into position to be shot and lines it up.
 * 
 * TODO: Time of Flight logic
 * 
 * @author Nicholas S, Michael F
 */
public class LineupTopCommand extends CommandBase {

  private final Delivery delivery;

  private final double DELIVERY_SLOW_SPEED = 0.1;
  
  public LineupTopCommand(Delivery delivery) {
    this.delivery = delivery;
    addRequirements(delivery);
  }

  @Override
  public void initialize() {
    // 
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    delivery.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}