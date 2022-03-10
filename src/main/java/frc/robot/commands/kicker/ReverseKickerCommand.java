package frc.robot.commands.kicker;

import frc.robot.subsystems.Kicker;

/**
 * Reverses the kicker motor after a delay.
 * @author Madison J.
 *
 */
public class ReverseKickerCommand extends WaitableKickerCommand {

  /**
   * Reverse the kicker motor
   */
  public ReverseKickerCommand(Kicker kicker) {
    this(0, kicker);
  }

  /**
   * Reverse the kicker motor after a delay.
   */
  public ReverseKickerCommand(double waitTimeSeconds, Kicker kicker) {
    super(kicker::reverse, waitTimeSeconds, kicker);
  }

}