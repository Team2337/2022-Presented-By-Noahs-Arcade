package frc.robot.commands.delivery;

import frc.robot.subsystems.Delivery;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetDeliverySpeed extends InstantCommand {
  private final Delivery subsystem;
  private final double speed;

  /**
   * @param subsystem The subsystem used by this command.
   * @param speed The speed (as a percent)
   */
  public SetDeliverySpeed(Delivery subsystem, double speed) {
    this.subsystem = subsystem;
    this.speed = speed;
    
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    //Sets delivery speed
    subsystem.setDeliverySpeed(speed);
  }

}