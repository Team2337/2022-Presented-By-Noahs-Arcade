package frc.robot.commands.shooter;

import frc.robot.subsystems.Kicker;

/**
 * Runs the kicker motor forward after a delay.
 * @author Madison J.
 *
 */
public class AutoKickerCommand extends AutoKickerWaitableCommand {

  /**
   * Runs the kicker motor forward after a delay.
   */
  public AutoKickerCommand(double waitTimeSeconds, Kicker kicker) {
    super(() -> kicker.start(0.5), waitTimeSeconds, kicker);
  }

}