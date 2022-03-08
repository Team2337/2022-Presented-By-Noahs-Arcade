package frc.robot.commands.shooter;

import frc.robot.subsystems.Kicker;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the kicker motor in reverse
 * @author Nicholas S
 *
 */
public class ReverseKicker extends CommandBase {

  private final Kicker kicker;

  public ReverseKicker(Kicker kicker) {
    this.kicker = kicker;

    addRequirements(kicker);
  }

  @Override
  public void execute() {
    kicker.reverse();
  }

  @Override
  public void end(boolean interrupted) {
    kicker.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}