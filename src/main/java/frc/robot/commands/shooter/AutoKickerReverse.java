package frc.robot.commands.shooter;

import frc.robot.subsystems.Kicker;

/**
 * Reverses the kicker motor after a delay.
 * @author Madison J.
 *
 */
public class AutoKickerReverse extends AutoKickerWaitableCommand {

  /**
   * Reverse the kicker motor after a delay.
   */
  public AutoKickerReverse(double waitTimeSeconds, Kicker kicker) {
    super(kicker::reverse, waitTimeSeconds, kicker);
  }

}