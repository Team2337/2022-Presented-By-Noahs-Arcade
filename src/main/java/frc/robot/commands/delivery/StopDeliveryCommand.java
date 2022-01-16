package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Delivery;

public class StopDeliveryCommand extends InstantCommand {
  private final Delivery subsystem;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public StopDeliveryCommand(Delivery subsystem) {
    this.subsystem = subsystem;
    
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    //Sets delivery speed
    subsystem.stopDelivery();
  }

}