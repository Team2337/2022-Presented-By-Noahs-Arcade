package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Stops the shooter
 *
 * @author Madison J.
 */
public class StopShooterInstantCommand extends InstantCommand {

  private final Shooter shooter;

  public StopShooterInstantCommand(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.stop();
  }

}