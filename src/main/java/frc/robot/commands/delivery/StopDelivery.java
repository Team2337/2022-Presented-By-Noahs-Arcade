package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Delivery;

public class StopDelivery extends InstantCommand {
  private final Delivery subsystem;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public StopDelivery(Delivery subsystem) {
    this.subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sets delivery speed
    subsystem.stopDelivery();
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stops motors
  }

}