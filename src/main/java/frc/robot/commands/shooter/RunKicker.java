package frc.robot.commands.shooter;

import frc.robot.subsystems.Kicker;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the kicker motor
 * @author Nicholas S
 *
 */
public class RunKicker extends CommandBase {

  private final Kicker kicker;

  public RunKicker(Kicker kicker) {
    this.kicker = kicker;

    addRequirements(kicker);
  }

  @Override
  public void execute() {
    kicker.start(kicker.kickerSpeedPercentageWidget.getDouble(0));
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