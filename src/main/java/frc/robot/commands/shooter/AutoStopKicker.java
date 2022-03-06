package frc.robot.commands.shooter;

import frc.robot.subsystems.Kicker;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Stops the kicker
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
    kicker.stop();
  }

}