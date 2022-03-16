package frc.robot.commands.kicker;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Runs the intake motor
 * @author Madison J.
 *
 */
public class StartKicker extends InstantCommand {

  private final Kicker kicker;

  public StartKicker(Kicker kicker) {
    this.kicker = kicker;

    addRequirements(kicker);
  }

  @Override
  public void initialize() {
    kicker.start(0.5);;
  }

}