package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 *Stops the shooter when a button is held and make sure it stops correctly
 *
 * @author Madison J.
 */
public class AutoStopShooter extends InstantCommand {

  private final Shooter shooter;

  public AutoStopShooter(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.stop();
  }
}