package frc.robot.commands.kicker;

import frc.robot.subsystems.Kicker;

/**
 * Runs the kicker motor forward after a delay.
 * @author Madison J.
 *
 */
public class ForwardKickerCommand extends WaitableKickerCommand {

  /**
   * Runs the kicker motor forward.
   */
  public ForwardKickerCommand(Kicker kicker) {
    this(0, kicker);
  }

  /**
   * Runs the kicker motor forward after a delay.
   */
  public ForwardKickerCommand(double waitTimeSeconds, Kicker kicker) {
    super(() -> kicker.start(0.5), waitTimeSeconds, kicker);
  }

}