package frc.robot.commands.delivery;

import frc.robot.subsystems.Delivery;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** An example command that uses an example subsystem. */
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
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sets delivery speed
    subsystem.setDeliverySpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}