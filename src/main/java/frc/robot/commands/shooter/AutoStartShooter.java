package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 *Runs the shooter when a button is held and make sure it stops correctly
 *
 * @author Madison J.
 */
public class AutoStartShooter extends InstantCommand {

  private final Shooter shooter;

  public AutoStartShooter(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setSpeed(40.7);
  }
}