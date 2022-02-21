package frc.robot.commands.shooter;

import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 *Runs the shooter when a button is held and make sure it stops correctly
 *
 * @author Madison J.
 */
public class AutoStopKicker extends InstantCommand {

  private final Kicker kicker;

  public AutoStopKicker(Kicker kicker) {
    this.kicker = kicker;

    addRequirements(kicker);
  }

  @Override
  public void initialize() {
    kicker.stop();;
  }
}